function [a_cmd, nu_next, diagnostic] = height_ista_tuned_acceleration(s, nu, parameters, vz, vd_raw, c33, plant)
%#codegen
% NED高度改进隐式ISTA：[k1 k2 epsilon amin amax nu_max Ts lambda vlim]。
% 保留限速前馈、重力倾斜补偿、物理推力约束和条件积分。
k1=parameters(1);k2=parameters(2);epsilon=parameters(3);
amin=parameters(4);amax=parameters(5);nu_max=parameters(6);
h=parameters(7);lambda=parameters(8);vlim=parameters(9);
m=plant(1);g=plant(2);kf=plant(3);CR=plant(4);omega0=plant(5);
a_ff=0;if abs(vd_raw)<vlim,a_ff=-lambda*vz;end
if epsilon>0
 [u_ista,nu_candidate]=ista_compensated_step(s,nu,k1,k2,h,epsilon);
else
 [u_ista,nu_candidate]=ISTA_author_step(-s,nu,k1,k2,h,0);
end
a_raw=a_ff+u_ista;
ctilt=min(max(c33,.2),1);Tmax=4*kf*(CR+omega0)^2;
lo=max(amin,g-Tmax*ctilt/m);hi=min(amax,g);
a_cmd=min(max(a_raw,lo),hi);
dnu=nu_candidate-nu;
if (a_raw>hi&&dnu>0)||(a_raw<lo&&dnu<0),nu_candidate=nu;end
nu_next=min(max(nu_candidate,-nu_max),nu_max);
diagnostic=[s;nu;a_raw;a_ff];
end
