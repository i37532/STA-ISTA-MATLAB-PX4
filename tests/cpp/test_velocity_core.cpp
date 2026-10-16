#include "velocity_sta_ista.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace velocity_control;
static int failures = 0;

static void check(bool ok, const std::string& message)
{
    if (!ok) { ++failures; std::cerr << "FAIL: " << message << '\n'; }
}

template<typename T>
static bool near(T actual, T expected, T tolerance)
{ return finite(actual) && std::abs(actual - expected) <= tolerance * (T(1) + std::abs(expected)); }

template<typename T> static void unit_tests()
{
    const T tolerance = sizeof(T) == sizeof(float) ? T(3e-5) : T(3e-11);
    Parameters<T> p = {T(3), T(1), T(.1), T(-100), T(100), T(5)};
    const T dt = T(.005);
    for (int mode = 0; mode != 2; ++mode) {
        const Algorithm algorithm = static_cast<Algorithm>(mode);
        const std::string tag = (mode ? "ISTA " : "STA ") + std::string(sizeof(T) == 4 ? "float: " : "double: ");
        State<T> zero;
        const Proposal<T> origin = step(algorithm, p, zero, Input<T>{T(0), T(0), dt});
        check(origin.valid() && origin.acceleration_limited == T(0) && zero.nu == T(0), tag + "origin");

        State<T> positive(T(.2)), negative(T(-.2));
        const Proposal<T> pos = step(algorithm, p, positive, Input<T>{T(.7), T(.3), dt});
        const Proposal<T> neg = step(algorithm, p, negative, Input<T>{T(-.7), T(-.3), dt});
        check(pos.valid() && neg.valid() && pos.acceleration_raw > T(0), tag + "positive error");
        check(near(pos.acceleration_raw, -neg.acceleration_raw, tolerance) &&
              near(positive.nu, -negative.nu, tolerance), tag + "odd symmetry");

        const T boundary = mode ? p.epsilon + dt*p.k1*std::sqrt(p.epsilon) + dt*dt*p.k2 : p.epsilon;
        for (int sign = -1; sign <= 1; sign += 2) {
            State<T> state;
            const T center = T(sign) * boundary;
            const Proposal<T> inside = propose(algorithm, p, state, Input<T>{center*T(.99999), T(0), dt});
            const Proposal<T> exact = propose(algorithm, p, state, Input<T>{center, T(0), dt});
            const Proposal<T> outside = propose(algorithm, p, state, Input<T>{center*T(1.00001), T(0), dt});
            check(inside.valid() && exact.valid() && outside.valid(), tag + "boundary valid");
            check(near(inside.acceleration_raw, outside.acceleration_raw, T(5e-5)), tag + "both boundaries continuous");
            if (mode) {
                check(near(exact.predicted_error, T(sign)*p.epsilon, tolerance), tag + "implicit boundary position");
                check(near(outside.predicted_error,
                           center*T(1.00001)-dt*(outside.acceleration_raw-state.nu), tolerance),
                      tag + "compensated implicit identity");
            }
        }

        Parameters<T> tight = p;
        tight.acceleration_min = T(-1); tight.acceleration_max = T(1);
        State<T> saturated(T(.2));
        const Proposal<T> clip = step(algorithm, tight, saturated, Input<T>{T(1), T(0), dt});
        check(clip.valid() && clip.output_limited && clip.acceleration_limited == T(1), tag + "upper output clamp");
        check(clip.integration_frozen_local && saturated.nu == T(.2), tag + "freeze worsening saturation");
        const Proposal<T> release = step(algorithm, tight, saturated, Input<T>{T(-.01), T(3), dt});
        check(release.valid() && release.output_limited && !release.integration_frozen_local && saturated.nu < T(.2),
              tag + "allow unwinding while still saturated");
        const Proposal<T> free = step(algorithm, tight, saturated, Input<T>{T(-.01), T(0), dt});
        check(free.valid() && !free.output_limited, tag + "release from saturation");

        State<T> lower(T(-.2));
        const Proposal<T> clip_low = step(algorithm, tight, lower, Input<T>{T(-1), T(0), dt});
        check(clip_low.valid() && clip_low.acceleration_limited == T(-1) &&
              clip_low.integration_frozen_local && lower.nu == T(-.2), tag + "lower clamp and freeze");

        Parameters<T> state_cap = p; state_cap.integral_max = T(.1);
        State<T> integral(T(.0999));
        const Proposal<T> capped = step(algorithm, state_cap, integral, Input<T>{T(1), T(0), dt});
        check(capped.valid() && capped.state_limited && integral.nu == T(.1), tag + "integral clamp");

        State<T> deferred(T(.2));
        Proposal<T> proposal = propose(algorithm, p, deferred, Input<T>{T(.5), T(.1), dt});
        check(deferred.nu == T(.2) && proposal.valid(), tag + "propose has no state side effect");
        check(commit(deferred, proposal, false) && deferred.nu == T(.2) &&
              proposal.integration_frozen_external, tag + "external coupled-limit hold");
        check(!commit(deferred, proposal, false), tag + "reject double commit");
        Proposal<T> stale = propose(algorithm, p, deferred, Input<T>{T(.5), T(0), dt});
        deferred.nu += T(.01);
        check(!commit(deferred, stale) && stale.status == Status::StaleProposal, tag + "reject stale state");
        deferred.reset(); check(deferred.nu == T(0), tag + "reset");

        const T bad_dt[] = {T(0), T(-.005), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::quiet_NaN()};
        for (T bad : bad_dt) {
            State<T> keep(T(.25));
            const Proposal<T> result = step(algorithm, p, keep, Input<T>{T(1), T(0), bad});
            check(!result.valid() && keep.nu == T(.25) && std::isnan(result.acceleration_limited), tag + "invalid dt preserves state");
        }
        const T invalid_value = std::numeric_limits<T>::quiet_NaN();
        State<T> keep(T(.25));
        check(!step(algorithm, p, keep, Input<T>{invalid_value, T(0), dt}).valid() && keep.nu == T(.25), tag + "invalid error");
        check(!step(algorithm, p, keep, Input<T>{T(0), invalid_value, dt}).valid() && keep.nu == T(.25), tag + "invalid feedforward");
        Parameters<T> invalid_p = p; invalid_p.epsilon = T(0);
        check(!step(algorithm, invalid_p, keep, Input<T>{T(0), T(0), dt}).valid() && keep.nu == T(.25), tag + "invalid parameters");
        Parameters<T> overflow = p; overflow.k1 = std::numeric_limits<T>::max();
        const Proposal<T> huge = step(algorithm, overflow, keep, Input<T>{T(100), T(0), T(2)});
        check(!huge.valid() && keep.nu == T(.25), tag + "numerical failure preserves state");
    }
}

template<typename T> static void compare_row(const std::vector<double>& v, std::size_t line)
{
    Parameters<T> p = {T(v[1]), T(v[2]), T(v[3]), T(v[4]), T(v[5]), T(v[6])};
    State<T> state{T(v[9])};
    const Proposal<T> got = step(static_cast<Algorithm>(int(v[0])), p, state,
                                 Input<T>{T(v[8]), T(v[10]), T(v[7])});
    const T tolerance = sizeof(T) == 4 ? T(5e-5) : T(5e-10);
    const std::string tag = "oracle row " + std::to_string(line) + (sizeof(T)==4 ? " float" : " double");
    check(got.valid(), tag + " valid");
    check(near(got.acceleration_raw, T(v[11]), tolerance), tag + " raw");
    check(near(got.acceleration_limited, T(v[12]), tolerance), tag + " limited");
    check(near(state.nu, T(v[13]), tolerance), tag + " next");
}

static void oracle_tests(const char* path)
{
    std::ifstream file(path);
    check(file.good(), std::string("MATLAB oracle CSV exists: ") + path);
    if (!file.good()) { return; }
    std::string line; std::getline(file, line); // ASCII/UTF-8 header (BOM allowed)
    std::size_t rows = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line == "\r") { continue; }
        ++rows;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream input(line);
        std::vector<double> values;
        double value;
        while (input >> value) { values.push_back(value); }
        check(values.size() == 14, "oracle row has 14 numeric fields");
        if (values.size() == 14) { compare_row<double>(values, rows+1); compare_row<float>(values, rows+1); }
    }
    check(rows >= 10, "oracle contains at least 10 cases");
    std::cout << "MATLAB oracle cases checked (double and float): " << rows << '\n';
}

int main(int argc, char** argv)
{
    if (argc == 1) { unit_tests<double>(); unit_tests<float>(); }
    else if (argc == 3 && std::string(argv[1]) == "--oracle") { oracle_tests(argv[2]); }
    else { std::cerr << "Usage: test_velocity_core [--oracle path.csv]\n"; return EXIT_FAILURE; }
    std::cout << (failures == 0 ? "PASS" : "FAIL") << ": " << failures << " failed checks\n";
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
