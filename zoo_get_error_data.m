function data4sysid = zoo_get_error_data(template_system,real_system,datafile_name)
%zoo_get_error_data: Generates training and validation data for a residual
%model given a template model and real system model
%   Only works for Arm systems where both models have the same input and output dimensions

% load in the template model
if isempty( template_system )
    [ datafile_name , datafile_path ] = uigetfile( 'systems/*.mat' , 'Choose template model...' );
	foo = load( [datafile_path , datafile_name] );
else
    foo = load(['systems' , filesep , template_system , filesep , template_system , '.mat']);
end
sys_temp = foo.Arm;

% load in the real system model
if isempty( template_system )
    [ datafile_name , datafile_path ] = uigetfile( 'systems/*.mat' , 'Choose real model...' );
	foo = load( [datafile_path , datafile_name] );
else
    foo = load(['systems' , filesep , real_system , filesep , real_system , '.mat']);
end
sys_real = foo.Arm;

%% set parameters
saveon = true;  % save a datafile if this is true
num_snaps_train = 1000; % number of snapshots to generate for training
num_snaps_val = 200;    % number of snapshots to generate for validation
Ts = 0.1;   % length of timestep
x0_min = -pi;    % maximum angle and angular velocity
x0_max = pi;     % minimum angle and angular velocity
u_min = -pi;    % minimum input
u_max = pi;     % maximum input

%% generate training data

% set inititial conditions
rand_vec_xreal = rand( num_snaps_train , sys_real.params.nx );
x_before_real = (1 - rand_vec_xreal) * x0_min + rand_vec_xreal * x0_max;

rand_vec_xtemp = rand( num_snaps_train , sys_temp.params.nx );
x_before_temp = (1 - rand_vec_xtemp) * x0_min + rand_vec_xtemp * x0_max;

% set inputs (for "real" system and template model)
rand_vec_ureal = rand( num_snaps_train , sys_real.params.nu );
ureal = (1 - rand_vec_ureal) * u_min + rand_vec_ureal * u_max;
rand_vec_utemp = rand( num_snaps_train , sys_temp.params.nu );
utemp = (1 - rand_vec_utemp) * u_min + rand_vec_utemp * u_max;

% compute response of the system for both models
x_after_real = zeros( num_snaps_train , sys_real.params.nx );
x_after_temp = zeros( num_snaps_train , sys_temp.params.nx );
for i = 1 : num_snaps_train
    x_after_real(i,:) = sys_real.simulate_Ts( x_before_real(i,:)' , ureal(i,:)' , [] , Ts )';
    x_after_temp(i,:) = sys_temp.simulate_Ts( x_before_temp(i,:)' , utemp(i,:)' , [] , Ts )';
end

% create the unlifted state of the residual system [error; yreal_k ; ureal_k]
y_before_real = sys_real.get_y( x_before_real );
y_before_temp = sys_temp.get_y( x_before_temp );
y_after_real = sys_real.get_y( x_after_real );
y_after_temp = sys_temp.get_y( x_after_temp );
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
rand_vec_xreal = rand( num_snaps_val , sys_real.params.nx );
x_before_real = (1 - rand_vec_xreal) * x0_min + rand_vec_xreal * x0_max;

rand_vec_xtemp = rand( num_snaps_val , sys_temp.params.nx );
x_before_temp = (1 - rand_vec_xtemp) * x0_min + rand_vec_xtemp * x0_max;

% set inputs (for "real" system and template model)
rand_vec_ureal = rand( num_snaps_val , sys_real.params.nu );
ureal = (1 - rand_vec_ureal) * u_min + rand_vec_ureal * u_max;
rand_vec_utemp = rand( num_snaps_val , sys_temp.params.nu );
utemp = (1 - rand_vec_utemp) * u_min + rand_vec_utemp * u_max;

% compute response of the system for both models
x_after_real = zeros( num_snaps_val , sys_real.params.nx );
x_after_temp = zeros( num_snaps_val , sys_temp.params.nx );
for i = 1 : num_snaps_val
    x_after_real(i,:) = sys_real.simulate_Ts( x_before_real(i,:)' , ureal(i,:)' , [] , Ts )';
    x_after_temp(i,:) = sys_temp.simulate_Ts( x_before_temp(i,:)' , utemp(i,:)' , [] , Ts )';
end

% create the unlifted state of the residual system [error; yreal_k ; ureal_k]
y_before_real = sys_real.get_y( x_before_real );
y_before_temp = sys_temp.get_y( x_before_temp );
y_after_real = sys_real.get_y( x_after_real );
y_after_temp = sys_temp.get_y( x_after_temp );
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
    if ~isempty(datafile_name)
        data4sysid = Data.get_data4sysid( train , val , true , datafile_name );
    else    
        data4sysid = Data.get_data4sysid( train , val , true , 'error-snapshots' );
    end
end

end







