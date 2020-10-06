% zoo_generate_snapshot_data
%
% Simulate the pendulum systems from a bunch of different initial
% conditions to get snapshot data

% System: Single pendulum with end effector position as the state

%% load in the real and template models for a single pendulum system
foo = load(['systems' , filesep , 'single-pend2_1-mods_1-links_20hz' , filesep , 'single-pend2_1-mods_1-links_20hz.mat']);
Arm_real = foo.Arm;
foo = load(['systems' , filesep , 'single-pend3_1-mods_1-links_20hz' , filesep , 'single-pend3_1-mods_1-links_20hz.mat']);
Arm_temp = foo.Arm;

%% set parameters
saveon = true;  % save a datafile if this is true
num_snaps_train = 1000; % number of snapshots to generate for training
num_snaps_val = 100;    % number of snapshots to generate for validation
Ts = 0.1;   % length of timestep
x0_min = -pi;    % maximum angle and angular velocity
x0_max = pi;     % minimum angle and angular velocity
u_min = -pi;    % minimum input
u_max = pi;     % maximum input

%% generate training data

% set inititial conditions
rand_vec_xreal = rand( num_snaps_train , 2 );
x_before_real = (1 - rand_vec_xreal) * x0_min + rand_vec_xreal * x0_max;

rand_vec_xtemp = rand( num_snaps_train , 2 );
x_before_temp = (1 - rand_vec_xtemp) * x0_min + rand_vec_xtemp * x0_max;

% set inputs (for "real" system and template model)
rand_vec_ureal = rand( num_snaps_train , 1 );
ureal = (1 - rand_vec_ureal) * u_min + rand_vec_ureal * u_max;
rand_vec_utemp = rand( num_snaps_train , 1 );
utemp = (1 - rand_vec_utemp) * u_min + rand_vec_utemp * u_max;

% compute response of the system for both models
x_after_real = zeros( num_snaps_train , 2 );
x_after_temp = zeros( num_snaps_train , 2 );
for i = 1 : num_snaps_train
    x_after_real(i,:) = Arm_real.simulate_Ts( x_before_real(i,:)' , ureal(i,:)' , [] , Ts )';
    x_after_temp(i,:) = Arm_temp.simulate_Ts( x_before_temp(i,:)' , utemp(i,:)' , [] , Ts )';
end

% create the unlifted state of the residual system [error; yreal_k ; ureal_k]
y_before_real = Arm_real.get_y( x_before_real );
y_before_temp = Arm_temp.get_y( x_before_temp );
y_after_real = Arm_real.get_y( x_after_real );
y_after_temp = Arm_temp.get_y( x_after_temp );
e_before = y_before_real - y_before_temp;   % error before
e_after = y_after_real - y_after_temp;   % error after

y_before = [ e_before , y_before_real , utemp ];
y_after = [ e_after , y_before_real , utemp ];
u = ureal - utemp;  % difference between inputs

% save the data in a format that Ksysid can handle
train = cell(1,1);
train{1}.y_before = y_before;
train{1}.y_after = y_after;
train{1}.y = y_after;   % save copy under this name for compatibility purposes
train{1}.u = u;
train{1}.Ts = Ts;   % need to save this value in all of the trials

%% generate validation data
% set inititial conditions
rand_vec_xreal = rand( num_snaps_val , 2 );
x_before_real = (1 - rand_vec_xreal) * x0_min + rand_vec_xreal * x0_max;

rand_vec_xtemp = rand( num_snaps_val , 2 );
x_before_temp = (1 - rand_vec_xtemp) * x0_min + rand_vec_xtemp * x0_max;

% set inputs (for "real" system and template model)
rand_vec_ureal = rand( num_snaps_val , 1 );
ureal = (1 - rand_vec_ureal) * u_min + rand_vec_ureal * u_max;
rand_vec_utemp = rand( num_snaps_val , 1 );
utemp = (1 - rand_vec_utemp) * u_min + rand_vec_utemp * u_max;

% compute response of the system for both models
x_after_real = zeros( num_snaps_val , 2 );
x_after_temp = zeros( num_snaps_val , 2 );
for i = 1 : num_snaps_val
    x_after_real(i,:) = Arm_real.simulate_Ts( x_before_real(i,:)' , ureal(i,:)' , [] , Ts )';
    x_after_temp(i,:) = Arm_temp.simulate_Ts( x_before_temp(i,:)' , utemp(i,:)' , [] , Ts )';
end

% create the unlifted state of the residual system [error; yreal_k ; ureal_k]
y_before_real = Arm_real.get_y( x_before_real );
y_before_temp = Arm_temp.get_y( x_before_temp );
y_after_real = Arm_real.get_y( x_after_real );
y_after_temp = Arm_temp.get_y( x_after_temp );
e_before = y_before_real - y_before_temp;   % error before (=0)
e_after = y_after_real - y_after_temp;   % error after

y_before = [ e_before , y_before_real , utemp ];
y_after = [ e_after , y_before_real , utemp ];
u = ureal - utemp;  % difference between inputs

% save the data in a format that Ksysid can handle
val = cell(1,1);
val{1}.y_before = y_before;
val{1}.y_after = y_after;
val{1}.y = y_after;   % save copy under this name for compatibility purposes
val{1}.u = u;
val{1}.Ts = Ts;   % need to save this value in all of the trials


%% Save data file

if saveon
    data4sysid = Data.get_data4sysid( train , val , true , 'pend-error-diff-ICs' );
end
















