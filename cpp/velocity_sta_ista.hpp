#ifndef VELOCITY_STA_ISTA_HPP
#define VELOCITY_STA_ISTA_HPP

#include <cmath>
#include <limits>
#include <type_traits>

// Standalone local-NED velocity controller core. No PX4 dependencies.
// The caller supplies acceleration feedforward explicitly. No gravity,
// attitude, thrust/PWM conversion, position gain or PX4 integrator is here.
namespace velocity_control {

enum class Algorithm { STA = 0, ISTA = 1 };
enum class Status { Valid, InvalidInput, NumericalFailure, StaleProposal };

template<typename T> struct Parameters {
    T k1, k2, epsilon, acceleration_min, acceleration_max, integral_max;
};

template<typename T> struct State {
    T nu;
    State() : nu(T(0)) {}
    explicit State(T initial) : nu(initial) {}
    void reset() { nu = T(0); }
};

template<typename T> struct Input {
    T error;                 // velocity_setpoint - velocity_measured [m/s]
    T acceleration_ff;       // explicit acceleration feedforward [m/s^2]
    T dt;                    // actual controller update interval [s]
};

template<typename T> struct Proposal {
    Status status;
    T acceleration_raw;
    T acceleration_limited;
    T nu_candidate;          // before conditional integration/state clamp
    T nu_next;               // after local anti-windup; updated by step()
    T nu_before;
    T predicted_error;       // ISTA prediction, not the next plant state
    bool output_limited;
    bool integration_frozen_local;
    bool integration_frozen_external;
    bool state_limited;
    bool committed;

    Proposal() : status(Status::InvalidInput),
        acceleration_raw(std::numeric_limits<T>::quiet_NaN()),
        acceleration_limited(std::numeric_limits<T>::quiet_NaN()),
        nu_candidate(std::numeric_limits<T>::quiet_NaN()),
        nu_next(std::numeric_limits<T>::quiet_NaN()),
        nu_before(std::numeric_limits<T>::quiet_NaN()),
        predicted_error(std::numeric_limits<T>::quiet_NaN()),
        output_limited(false), integration_frozen_local(false),
        integration_frozen_external(false), state_limited(false), committed(false) {}
    bool valid() const { return status == Status::Valid; }
};

template<typename T> inline T clamp(T x, T lo, T hi)
{ return x < lo ? lo : (x > hi ? hi : x); }

template<typename T> inline bool finite(T x) { return std::isfinite(x); }

// Side-effect free: the integrator is not changed until commit().
template<typename T>
Proposal<T> propose(Algorithm algorithm, const Parameters<T>& p,
                    const State<T>& state, const Input<T>& in)
{
    static_assert(std::is_floating_point<T>::value, "Use float or double.");
    Proposal<T> result;
    if ((algorithm != Algorithm::STA && algorithm != Algorithm::ISTA) ||
        !finite(p.k1) || !finite(p.k2) || !finite(p.epsilon) ||
        !finite(p.acceleration_min) || !finite(p.acceleration_max) ||
        !finite(p.integral_max) || !finite(state.nu) ||
        !finite(in.error) || !finite(in.acceleration_ff) || !finite(in.dt) ||
        p.k1 < T(0) || p.k2 < T(0) || p.epsilon <= T(0) ||
        p.acceleration_min > p.acceleration_max ||
        p.integral_max < T(0) || in.dt <= T(0)) {
        return result;
    }

    T control, candidate, predicted = in.error;
    if (algorithm == Algorithm::STA) {
        const T sigma = clamp(in.error / p.epsilon, T(-1), T(1));
        const T magnitude = std::abs(in.error) > p.epsilon
            ? std::abs(in.error) : p.epsilon;
        control = p.k1 * std::sqrt(magnitude) * sigma + state.nu;
        candidate = state.nu + in.dt * p.k2 * sigma;
    } else {
        // Compensated implicit predictor:
        // predicted = error - dt * (u_ista - nu), so w = error.
        const T A = in.dt * p.k1;
        const T B = in.dt * in.dt * p.k2;
        const T root_epsilon = std::sqrt(p.epsilon);
        const T boundary = p.epsilon + A * root_epsilon + B;
        T sigma, psi;
        if (!finite(A) || !finite(B) || !finite(boundary)) {
            result.status = Status::NumericalFailure;
            return result;
        }
        if (std::abs(in.error) <= boundary) {
            const T denominator = T(1) + A/root_epsilon + B/p.epsilon;
            if (!finite(denominator)) {
                result.status = Status::NumericalFailure;
                return result;
            }
            predicted = in.error / denominator;
            sigma = predicted / p.epsilon;
            psi = predicted / root_epsilon;
        } else {
            const T C = std::abs(in.error) - B;
            // Rationalized quadratic root, matching MATLAB oracle.
            const T discriminant = A*A + T(4)*C;
            const T denominator = std::sqrt(discriminant) + A;
            const T root = T(2)*C / denominator;
            sigma = in.error > T(0) ? T(1) : T(-1);
            predicted = sigma * root * root;
            psi = sigma * root;
            if (!finite(discriminant) || !finite(denominator) || !finite(root)) {
                result.status = Status::NumericalFailure;
                return result;
            }
        }
        candidate = state.nu + in.dt * p.k2 * sigma;
        control = p.k1 * psi + candidate;
    }

    const T raw = in.acceleration_ff + control;
    if (!finite(raw) || !finite(control) || !finite(candidate) || !finite(predicted)) {
        result.status = Status::NumericalFailure;
        return result;
    }
    const T limited = clamp(raw, p.acceleration_min, p.acceleration_max);
    const T increment = candidate - state.nu;
    const bool freeze =
        (raw > p.acceleration_max && increment > T(0)) ||
        (raw < p.acceleration_min && increment < T(0));
    const T eligible = freeze ? state.nu : candidate;
    const T next = clamp(eligible, -p.integral_max, p.integral_max);
    result.status = Status::Valid;
    result.acceleration_raw = raw;
    result.acceleration_limited = limited;
    result.nu_candidate = candidate;
    result.nu_next = next;
    result.nu_before = state.nu;
    result.predicted_error = predicted;
    result.output_limited = raw != limited;
    result.integration_frozen_local = freeze;
    result.state_limited = eligible != next;
    return result;
}

// Call after the downstream PX4 thrust/tilt/allocation limits are known.
// external_integrate=false holds the current state exactly. The caller must
// derive this per-axis flag from the downstream coupled saturation logic;
// independent acceleration clamps cannot replace that logic.
// A proposal must be committed once, against the state used to create it.
template<typename T>
bool commit(State<T>& state, Proposal<T>& proposal, bool external_integrate = true)
{
    if (!proposal.valid()) { return false; }
    if (proposal.committed || !finite(state.nu) || state.nu != proposal.nu_before) {
        proposal.status = Status::StaleProposal;
        return false;
    }
    if (!finite(proposal.nu_next)) {
        proposal.status = Status::NumericalFailure;
        return false;
    }
    if (!external_integrate) {
        proposal.nu_next = state.nu;
        proposal.integration_frozen_external = true;
        proposal.state_limited = false;
    }
    state.nu = proposal.nu_next;
    proposal.committed = true;
    return true;
}

// Convenience one-axis step: local limits only unless caller passes false.
// Invalid inputs/numerics leave state unchanged and return non-valid status;
// NaN output diagnostics are intentionally NOT a zero-acceleration fallback.
template<typename T>
Proposal<T> step(Algorithm algorithm, const Parameters<T>& p,
                 State<T>& state, const Input<T>& in,
                 bool external_integrate = true)
{
    Proposal<T> result = propose(algorithm, p, state, in);
    commit(state, result, external_integrate);
    return result;
}

} // namespace velocity_control
#endif
