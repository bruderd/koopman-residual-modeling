%zoo_close_model_gap_pendulum_example_simpler

%% Load the single and double pendulum arm models
temp = load(['systems' , filesep , 'single-pend2_1-mods_1-links_20hz' , filesep , 'single-pend2_1-mods_1-links_20hz.mat']);
Arm_double = temp.Arm;
temp = load(['systems' , filesep , 'single-pend_1-mods_1-links_20hz' , filesep , 'single-pend_1-mods_1-links_20hz.mat']);
Arm_single = temp.Arm;

%% Load in the training data for the two pendulum systems
data4sysid_double = load(['datafiles' , filesep , 'single-pend2_1-links_endeff_train-2_val-2_2020-09-17_18-38.mat']);   % this is the real system
data4sysid_single = load(['datafiles' , filesep , 'single-pend_1-links_endeff_train-2_val-2_2020-09-17_18-00.mat']);    % this is the system model

%% Create the error state and input
train = cell(2,1);
val = cell(2,1);

for i = 1 : 2
train{i}.t = data4sysid_double.train{i}.t;
val{i}.t = data4sysid_double.val{i}.t;

% Simulate the model one time step give real model and real input

% training data
for j = 1 : floor( length( data4sysid_double.train{i}.t ) / 2 )
    x_k = data4sysid_double.train{i}.x(2*j-1,:)';
    u_k = data4sysid_single.train{i}.u(2*j-1,:)';   % different input applied to model
    x_kp1 = Arm_single.simulate_Ts( x_k , u_k , [] );
    
    train{i}.x(2*j-1,:) = x_k';
    train{i}.x(2*j,:) = x_kp1';
end
train_model_y = Arm_single.get_y( train{i}.x );

% validation data
for j = 1 : floor( length( data4sysid_double.val{i}.t ) / 2 )
    x_k = data4sysid_double.val{i}.x(2*j-1,:)';
    u_k = data4sysid_double.val{i}.u(2*j-1,:)';
    x_kp1 = Arm_single.simulate_Ts( x_k , u_k , [] );
    
    val{i}.x(2*j-1,:) = x_k';
    val{i}.x(2*j,:) = x_kp1';
end
val_model_y = Arm_single.get_y( val{i}.x );

% create snapshot pairs for training
train{i}.snaphshots.alpha = [ data4sysid_double.train{i}.y(1:2:end,:) - train{i}.x(1:2:end,:) , train{i}.x(1:2:end,:) , data4sysid_double.train{i}.u(1:2:end,:) ];
train{i}.snapshots.beta = [ data4sysid_double.train{i}.y(2:2:end,:) - train{i}.x(2:2:end,:) , train{i}.x(1:2:end,:) , data4sysid_double.train{i}.u(1:2:end,:) ];
train{i}.snapshots.u = [ data4sysid_double.train{i}.u(1:2:end,:) - data4sysid_single.train{i}.u(1:2:end,:) ];

% THIS IS TRASH, FIELDS ONLY CREATED SO KSYSID WILL ACCEPT THEM
train{i}.y = [ data4sysid_double.train{i}.y - train_model_y , data4sysid_double.train{i}.y , data4sysid_double.train{i}.u ];
val{i}.y = [ data4sysid_double.val{i}.y - val_model_y , data4sysid_double.val{i}.y , data4sysid_double.val{i}.u ];

train{i}.u = data4sysid_double.train{i}.u - data4sysid_single.train{i}.u;
val{i}.u = data4sysid_double.val{i}.u - data4sysid_single.val{i}.u;
end


%% Create data file for error dynamics

data4sysid_error = Data.get_data4sysid( train , val , true , 'single-pendulum-error-dynamics' );