function [u_k, nu_next, r_k] = ISTA_author_step(x_k, nu_k, lambda_1, lambda_2, h, Delta_k)
% 按作者截图计算 u、nu，调用后执行 x_next = x_k + h*u_k。
% x、nu、u 分别对应此前代码的 x1、x2、f1_step。
% 此处 nu = nu_ctrl + phi 为闭环第二状态，nu_ctrl 才是原 ISTA.m 的积分状态；
% phi 为对象扰动，phi_dot = Delta。截图的 phi(t) 在此对应 Delta(t)。
%
% 输入：x_k, nu_k                 t_k 时刻的闭环状态
%       lambda_1, lambda_2, h     增益及时间步长
%       Delta_k                  扰动导数 Delta(t_k)
% 输出：u_k                      作者截图中的本步增量速率
%       nu_next                  t_{k+1} 时刻的第二状态
%       r_k                      名义隐式步的非负平方根幅值
%
% u_k 一般不等于图 7 的后处理量 u_plot = nu-lambda_1*sqrt(abs(x))*sign(x)，
% 也不直接等同于原对象的物理控制输入。本函数用于复现作者闭环仿真。
% 以下公式及运算顺序保持作者截图的写法。

    a = h * lambda_1;
    b_k = -x_k - h * nu_k;

    if b_k < -h^2 * lambda_2
        r_k = (-a + sqrt(a^2 - 4 * (b_k + lambda_2 * h^2))) / 2;
        nu_next = nu_k - lambda_2 * h + h * Delta_k;
        u_k = -lambda_1 * r_k + nu_next;
    elseif b_k > h^2 * lambda_2
        r_k = (-a + sqrt(a^2 + 4 * (b_k - lambda_2 * h^2))) / 2;
        nu_next = nu_k + lambda_2 * h + h * Delta_k;
        u_k = lambda_1 * r_k + nu_next;
    else
        r_k = 0;
        nu_next = nu_k + b_k / h + h * Delta_k;
        u_k = nu_next;
    end
end
