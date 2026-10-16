function export_controller_reference(output_file)
% 用原模型对应的 MATLAB 函数生成 C++ 单步一致性测试数据。
% 从本文件所在目录或 addpath 后运行 export_controller_reference。
% 算法编号：0=改进 STA，1=改进 ISTA。每行是独立的一次采样。
% 本测试不运行 Simulink，不代表飞机或 PX4 的闭环性能验证。
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab'));
cfg = load_controller_parameters;
if nargin < 1
    output_file = fullfile(root,'tests','matlab','controller_reference.csv');
end
fid = fopen(output_file,'w');
assert(fid >= 0,'Cannot open reference CSV.');
closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,['algorithm,k1,k2,epsilon,lo,hi,nu_max,dt,error,nu,accel_ff,' ...
    'expected_raw,expected_limited,expected_next\n']);
count=0;
for algorithm=0:1
    if algorithm==0, family=cfg.sta; else, family=cfg.ista; end
    for axis=1:2
        if axis==1, parameters=family.xy; else, parameters=family.z; end
        epsilon=parameters(3); lambda=parameters(8);
        for dt=[.0025 .005 .01]
            parameters(7)=dt;
            % 包括显式边界 epsilon 和隐式分支边界两侧。
            implicit_boundary=epsilon+dt*parameters(1)*sqrt(epsilon)+dt^2*parameters(2);
            positive=[1e-12 epsilon*(1-1e-9) epsilon epsilon*(1+1e-9) ...
                implicit_boundary*(1-1e-9) implicit_boundary ...
                implicit_boundary*(1+1e-9) .5 5];
            errors=unique([-positive 0 positive]);
            states=[-parameters(6) -parameters(6)/2 0 parameters(6)/2 parameters(6)];
            for error=errors
                for nu=states
                    for accel_ff=[-30 -3 -.75 0 .75 3 30]
                        velocity=-accel_ff/lambda;
                        lo=parameters(4); hi=parameters(5);
                        if axis==1
                            if algorithm==0
                                [limited,next]=sta_velocity_ff_conditional(error,nu,parameters,velocity);
                                sigma=min(max(error/epsilon,-1),1);
                                raw=accel_ff+parameters(1)*sqrt(max(abs(error),epsilon))*sigma+nu;
                            else
                                [limited,next]=ista_velocity_tuned(error,nu,parameters,velocity);
                                core=ista_compensated_step(error,nu,parameters(1),parameters(2),dt,epsilon);
                                raw=accel_ff+core;
                            end
                        else
                            % c33=1，vd_raw=0：前馈分支开启；s 是独立测试输入。
                            % lo/hi 写入实际物理约束后的值，供 C++ 通用核比较。
                            c33=1; vd_raw=0;
                            Tmax=4*cfg.plant(3)*(cfg.plant(4)+cfg.plant(5))^2;
                            lo=max(lo,cfg.plant(2)-Tmax*c33/cfg.plant(1));
                            hi=min(hi,cfg.plant(2));
                            if algorithm==0
                                [limited,next,diagnostic]=height_sta_acceleration(error,nu,parameters,velocity,vd_raw,c33,cfg.plant);
                            else
                                [limited,next,diagnostic]=height_ista_tuned_acceleration(error,nu,parameters,velocity,vd_raw,c33,cfg.plant);
                            end
                            raw=diagnostic(3);
                        end
                        row=[algorithm parameters(1:3) lo hi parameters(6) dt ...
                            error nu accel_ff raw limited next];
                        assert(all(isfinite(row)) && limited>=lo && limited<=hi);
                        assert(abs(next)<=parameters(6));
                        fprintf(fid,'%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n',row);
                        count=count+1;
                    end
                end
            end
        end
    end
end
fprintf('Exported %d original MATLAB wrapper reference cases: %s\n',count,output_file);
end
