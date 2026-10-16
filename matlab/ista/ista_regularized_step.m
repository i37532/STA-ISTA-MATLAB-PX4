function [u, nu_candidate, s_pred] = ista_regularized_step(s, nu, k1, k2, Ts, epsilon)
%#codegen
% 边界层隐式 ISTA 工程扩展；不是作者原函数的原样复制。
% s = 目标速度 - 实际速度，nu 是当前积分状态，Ts 是控制采样周期。
% epsilon > 0 是速度误差边界层宽度，单位与 s 相同。
%
% 将下面三个名义、未限幅的离散方程联合求解：
% s_pred       = s - Ts*u
% nu_candidate = nu + Ts*k2*sat(s_pred/epsilon)
% u            = k1*sqrt(max(abs(s_pred),epsilon))*sat(s_pred/epsilon)
%                + nu_candidate
%
% 外层保留作者 ISTA 的二次根，内层采用线性边界层的精确隐式解。
% epsilon 趋于零时恢复作者名义隐式核；s_pred 不是实际飞机下一步状态。
% 前馈、执行器动态、输出限幅和条件积分由调用者处理；实际饱和闭环
% 不再严格满足以上无约束递推，不能据此宣称无条件有限时间收敛。

assert(Ts > 0 && epsilon > 0);
assert(k1 >= 0 && k2 >= 0);

w = s - Ts*nu;
A = Ts*k1;
B = Ts*Ts*k2;
sqrt_epsilon = sqrt(epsilon);
boundary = epsilon + A*sqrt_epsilon + B;

if abs(w) <= boundary
    % |s_pred| <= epsilon：同时解误差更新与积分更新。
    denominator = 1 + A/sqrt_epsilon + B/epsilon;
    s_pred = w/denominator;
    sigma = s_pred/epsilon;
    psi = s_pred/sqrt_epsilon;
else
    % |s_pred| > epsilon：与作者外层解相同。
    % 有理化避免 sqrt(A^2+4*C)-A 两个相近数相减。
    C = abs(w) - B;
    r = 2*C/(sqrt(A*A + 4*C) + A);
    sigma = sign(w);
    s_pred = sigma*r*r;
    psi = sigma*r;
end

nu_candidate = nu + Ts*k2*sigma;
u = k1*psi + nu_candidate;
end
