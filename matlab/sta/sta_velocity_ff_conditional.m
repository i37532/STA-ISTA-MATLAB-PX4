function [u, v_next] = sta_velocity_ff_conditional(e, v, parameters, velocity)
%#codegen
% 水平速度环：带目标速度变化补偿、边界层和条件积分的离散 STA。
% e = v_desired - v_actual，velocity 是同一惯性系方向的实际速度。
% v 是控制器积分状态，不是飞机的实际速度。
% parameters = [k1 k2 epsilon umin umax vmax Ts lambda]。
% 本参数顺序用于 x/y 速度环；高度环使用独立的 height_sta_acceleration。
% 当位置目标在两次阶跃之间保持不变，v_desired=lambda*(pos_d-pos)，
% 因而 dot(v_desired)=-lambda*velocity。连续轨迹需要另行提供正确前馈。

k1 = parameters(1);
k2 = parameters(2);
epsilon = parameters(3);
umin = parameters(4);
umax = parameters(5);
vmax = parameters(6);
Ts = parameters(7);
lambda = parameters(8);

sigma = min(max(e / epsilon, -1), 1);
acceleration_ff = -lambda * velocity;
u_raw = acceleration_ff + k1 * sqrt(max(abs(e), epsilon)) * sigma + v;
u = min(max(u_raw, umin), umax);

% 条件积分：已饱和且积分还会加重饱和时，暂停积分。
% 若积分能够帮助退出饱和，则正常更新。
v_dot = k2 * sigma;
if (u_raw > umax && v_dot > 0) || (u_raw < umin && v_dot < 0)
    v_dot = 0;
end
v_next = min(max(v + Ts * v_dot, -vmax), vmax);
end
