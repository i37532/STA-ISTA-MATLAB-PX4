function [throttle, T_des] = height_acceleration_to_throttle(a_cmd,c33,plant)
%#codegen
% a_z=g-T*cos(phi)*cos(theta)/m；转为总推力，再转为公共油门。
% 此处使用稳态逆映射，原模型的 0.02 s 电机滞后仍然保留。
m=plant(1); g=plant(2); kf=plant(3); CR=plant(4); omega0=plant(5);
ctilt=min(max(c33,0.2),1);
T_des=max(0,m*(g-a_cmd)/ctilt);
omega_des=sqrt(T_des/(4*kf));
throttle=min(max((omega_des-omega0)/CR,0),1);
end
