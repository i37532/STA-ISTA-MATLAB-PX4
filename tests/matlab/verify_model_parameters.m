function verify_model_parameters
% 只加载临时改名副本读取参数；不仿真、不保存、不影响已打开的原模型。
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab'));
cfg=load_controller_parameters;
models={'sta','drone_position_control_sta_z_tuned';'ista','drone_position_control_ista_tuned'};
for i=1:2
    [~,token]=fileparts(tempname);
    name=['px4_check_' token];
    path=fullfile(tempdir,[name '.slx']);
    copyfile(fullfile(root,'models',models{i,1},[models{i,2} '.slx']),path);
    load_system(path);
    cleanup=onCleanup(@() close_system(name,0));
    mw=get_param(name,'ModelWorkspace');
    h=[name '/水平和高度控制'];
    if i==1
        assert(isequal(getVariable(mw,'HeightSTAParams'),cfg.sta.z));
        for ax={'x','y'}
            expression=get_param([h '/STA_' ax{1} '_velocity/parameters'],'Value');
            assert(isequal(eval(expression),cfg.sta.xy));
        end
    else
        assert(isequal(getVariable(mw,'HeightISTAParams'),cfg.ista.z));
        assert(isequal(getVariable(mw,'XYISTAParams'),cfg.ista.xy));
    end
    assert(isequal(getVariable(mw,'HeightNominalPlant'),cfg.plant));
    fprintf('JSON parameters match saved model: %s\n',models{i,2});
    clear cleanup
end
end
