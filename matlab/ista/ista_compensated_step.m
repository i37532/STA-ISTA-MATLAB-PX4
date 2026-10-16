function [u, nu_candidate, s_pred] = ista_compensated_step(s, nu, k1, k2, Ts, epsilon)
%#codegen
% 工程改进：用积分状态近似补偿未知加速度扰动，修正隐式预测。
% 假设局部误差动力学 s_dot=-u-d；nu在稳态提供-d的补偿。
% 因而采用 d_hat=-nu，预测 s_pred=s-Ts*(u-nu)。
% 与nu_next=nu+Ts*k2*sat(s_pred/epsilon)、u=k1*psi(s_pred)+nu_next联立。
% 把原核输入平移Ts*nu后，消元右端从s-Ts*nu变为s。
% 不读取实际扰动力；nu并不是任何瞬态下都准确的扰动观测器。
% 这是作者名义隐式核的工程扩展，不等同作者原函数。
[u,nu_candidate,s_pred]=ista_regularized_step(s+Ts*nu,nu,k1,k2,Ts,epsilon);
end
