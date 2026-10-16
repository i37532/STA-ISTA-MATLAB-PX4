function [u, nu_next] = ista_velocity_tuned(e, nu, parameters, velocity)
%#codegen
% 改进隐式ISTA：参数 [k1 k2 epsilon amin amax nu_max Ts lambda]。
% epsilon>0 使用边界层隐式解；epsilon=0 为原作者隐式核。
k1=parameters(1);k2=parameters(2);epsilon=parameters(3);
lo=parameters(4);hi=parameters(5);nu_max=parameters(6);
h=parameters(7);lambda=parameters(8);
if epsilon>0
 [u_ista,nu_candidate]=ista_compensated_step(e,nu,k1,k2,h,epsilon);
else
 [u_ista,nu_candidate]=ISTA_author_step(-e,nu,k1,k2,h,0);
end
u_raw=-lambda*velocity+u_ista;
u=min(max(u_raw,lo),hi);
dnu=nu_candidate-nu;
if (u_raw>hi&&dnu>0)||(u_raw<lo&&dnu<0)
 nu_candidate=nu;
end
nu_next=min(max(nu_candidate,-nu_max),nu_max);
end
