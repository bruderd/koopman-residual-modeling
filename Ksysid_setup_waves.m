% Ksysid_setup_waves
%
% Creates a sysid class and walks through all of the steps of building a
% model from data, validating its performance, and saving it (if desired)
% 
% Written as a function so it can be called in waves


%% gather training data (need to prepare data file before running this)

% % load in data file(s)
data4sysid = load([ 'datafiles' , filesep , 'thesis-3link-markers-noload-50trials_train-10_val-5_2020-06-04_17-04.mat' ]); % for thesis model comparison of 3link arm
% [ datafile_name , datafile_path ] = uigetfile( 'datafiles/*.mat' , 'Choose data file for sysid...' );
% data4sysid = load( [datafile_path , datafile_name] );


%% construct sysid class
Ksysid = Ksysid( data4sysid ,...
        'model_type' , 'bilinear' ,...    % model type (linear, bilinear, or nonlinear)
        'obs_type' , { 'poly' } ,...    % type of basis functions
        'obs_degree' , [ 4 ] ,...       % "degree" of basis functions
        'snapshots' , Inf ,...          % Number of snapshot pairs
        'lasso' , [ 2 ] ,...          % L1 regularization term
        'delays' , 0 ,...               % Numer of state/input delays
        'loaded' , false ,...           % Does system include loads?
        'dim_red' , false);             % Should dimensional reduction be performed?

if Ksysid.loaded
    disp(['Number of basis functions: ' , num2str( (Ksysid.params.nw + 1) * Ksysid.params.N ) ]);
else
   disp(['Number of basis functions: ' , num2str( Ksysid.params.N ) ]);
end
    
%% basis dimensional reduction

disp('Performing dimensional reduction...');
Px = Ksysid.lift_snapshots( Ksysid.snapshotPairs );
Ksysid = Ksysid.get_econ_observables( Px );
disp(['Number of basis functions: ' , num2str( Ksysid.params.N ) ]);
clear Px;
    
%% train model(s)
Ksysid = Ksysid.train_models;


%% validate model(s)
% could also manually do this for one model at a time

% results = cell( size(Ksysid.candidates) );    % store results in a cell array
% err = cell( size(Ksysid.candidates) );    % store error in a cell array 
% 
% if iscell(Ksysid.candidates)
%     for i = 1 : length(Ksysid.candidates)
%         [ results{i} , err{i} ] = Ksysid.valNplot_model( i );
%     end
% else
%     [ results{1} , err{1} ] = Ksysid.valNplot_model;
% end
%     
% % calculate aggregate error accross all trials
% toterr.mean = zeros( size(err{1}{1}.mean) );
% toterr.rmse = zeros( size(err{1}{1}.rmse) );
% toterr.nrmse = zeros( size(err{1}{1}.nrmse) );
% for i = 1:length(err{1})
%     toterr.mean = toterr.mean + err{1}{i}.mean; 
%     toterr.rmse = toterr.rmse + err{1}{i}.rmse;
%     toterr.nrmse = toterr.nrmse + err{1}{i}.nrmse;
% end

%% save model(s)

% You do this based on the validation results.
% Call this function:
  Ksysid.save_class( )