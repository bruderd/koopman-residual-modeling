%zoo_close_model_gap_pendulum_example

%% Load the single and double pendulum arm models
temp = load(['systems' , filesep , 'double-pend_2-mods_1-links_20hz' , filesep , 'double-pend_2-mods_1-links_20hz.mat']);
Arm_double = temp.Arm;
temp = load(['systems' , filesep , 'single-pend_1-mods_1-links_20hz' , filesep , 'single-pend_1-mods_1-links_20hz.mat']);
Arm_single = temp.Arm;

%% Load in the training data for the single and double pendulum systems
data4sysid_double = load(['datafiles' , filesep , 'double-pend_2-links_endeff_train-2_val-2_2020-09-17_17-19.mat']);
data4sysid_single = load(['datafiles' , filesep , 'single-pend_1-links_endeff_train-2_val-2_2020-09-17_18-00.mat']);

%% Create the error state and input
train = cell(2,1);
val = cell(2,1);

for i = 1 : 2
train{i}.t = data4sysid_double.train{i}.t;
val{i}.t = data4sysid_double.val{i}.t;
    
train{i}.y = data4sysid_double.train{i}.y - data4sysid_single.train{i}.y;
val{i}.y = data4sysid_double.val{i}.y - data4sysid_single.val{i}.y;

train{i}.u = data4sysid_double.train{i}.u - [ data4sysid_single.train{i}.u , zeros(size(data4sysid_single.train{i}.u,1),1) ];
val{i}.u = data4sysid_double.val{i}.u - [ data4sysid_single.val{i}.u , zeros(size(data4sysid_single.val{i}.u,1),1) ];
end

%% Create data file for error dynamics

data4sysid_error = Data.get_data4sysid( train , val , true , 'pendulum-error-dynamics' );