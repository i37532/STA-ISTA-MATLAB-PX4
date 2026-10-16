function cfg = load_controller_parameters
% 从包内 JSON 读取原 Simulink 整定参数，并添加包内参考函数路径。
% 不依赖当前目录，不修改基础工作区，也不改写模型参数。
here = fileparts(mfilename('fullpath'));
cfg = jsondecode(fileread(fullfile(here, '..', 'parameters', 'controller_parameters.json')));
cfg.sta.xy = cfg.sta.xy(:).';
cfg.sta.z = cfg.sta.z(:).';
cfg.ista.xy = cfg.ista.xy(:).';
cfg.ista.z = cfg.ista.z(:).';
cfg.plant = cfg.plant(:).';
addpath(fullfile(here,'sta'),fullfile(here,'ista'),fullfile(here,'common'));
end
