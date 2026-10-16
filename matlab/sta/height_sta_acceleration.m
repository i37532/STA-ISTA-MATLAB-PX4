function [a_cmd, nu_next, diagnostic] = height_sta_acceleration(s, nu, parameters, vz, vd_raw, c33, plant)
%#codegen
% 高度专用离散 STA：输出惯性系 z 向加速度，向下为正。
% s = sat(lambda*(z_d-z), +/-vlim) - vz，单位 m/s。
% nu 是加速度单位的积分状态，不是飞机速度。
% parameters = [k1 k2 epsilon amin amax nu_max Ts lambda vlim]。
% plant = [m_nom g kf CR omega0]，控制器标称值与实际质量分开。
% 仅对分段恒定位置目标使用本前馈；阶跃瞬间不计算冲激导数。
k1=parameters(1); k2=parameters(2); epsilon=parameters(3);
amin=parameters(4); amax=parameters(5); nu_max=parameters(6);
Ts=parameters(7); lambda=parameters(8); vlim=parameters(9);
m=plant(1); g=plant(2); kf=plant(3); CR=plant(4); omega0=plant(5);

% 目标速度限幅时是常数，此时前馈必须为零。
a_ff=0;
if abs(vd_raw)<vlim
    a_ff=-lambda*vz;
end
sigma=min(max(s/epsilon,-1),1);
a_raw=a_ff+k1*sqrt(max(abs(s),epsilon))*sigma+nu;

% 同时考虑设计加速度限幅、最大油门和非负推力。
ctilt=min(max(c33,0.2),1);
Tmax=4*kf*(CR+omega0)^2;
lo=max(amin,g-Tmax*ctilt/m);
hi=min(amax,g);
a_cmd=min(max(a_raw,lo),hi);

% 条件积分：饱和且积分方向加重饱和时暂停；反向积分仍允许。
nu_dot=k2*sigma;
if (a_raw>hi && nu_dot>0)||(a_raw<lo && nu_dot<0)
    nu_dot=0;
end
nu_next=min(max(nu+Ts*nu_dot,-nu_max),nu_max);
diagnostic=[s;nu;a_raw;a_ff];
end
