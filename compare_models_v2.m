% compare_models_v2.m
%
% Simulate a bunch of models on the same set of validation data and compute
% overall error

% % load in the data file with validation trials in it
% [ datafile_name , datafile_path ] = uigetfile( 'datafiles/*.mat' , 'Choose data file for validation...' );
% data = load( [datafile_path , datafile_name] );

% load in the set of model files 
[ modelfile_name , modelfile_path ] = uigetfile( 'systems/*.mat' , 'Choose model files...' , 'MultiSelect' , 'on' );

%% calculate error for each model
toterr = cell( size( modelfile_name ) );
for j = 1 : length( modelfile_name )
    temp = load( [modelfile_path , modelfile_name{j}] );
    Ksysid = temp.sysid_class;
    
    % run validation trials
    [ results , err ] = Ksysid.valNplot_model( [] , false );

    % save some useful info from the sysid class
    toterr{j}.model_type = Ksysid.model_type;
    toterr{j}.obs_type = Ksysid.obs_type;
    toterr{j}.obs_degree = Ksysid.obs_degree;
    toterr{j}.dimK = size( Ksysid.koopData.K , 1 );
%     if strcmp( Ksysid.model_type , 'bilinear' )
%         toterr{j}.dimK = length( Ksysid.basis.econ_full_input );
%         toterr{j}.dimK_big = length( Ksysid.basis.full_input );
%     else
%         toterr{j}.dimK = length( Ksysid.basis.econ_full );
%         toterr{j}.dimK_big = length( Ksysid.basis.full );
%     end
    
    % calculate aggregate error accross all trials
    toterr{j}.mean = zeros( size(err{1}.mean) );
    toterr{j}.rmse = zeros( size(err{1}.rmse) );
    toterr{j}.nrmse = zeros( size(err{1}.nrmse) );
    toterr{j}.euclid = [];
    toterr{j}.euclid_zsr = [];
    toterr{j}.unscaled.euclid = [];
    toterr{j}.unscaled.euclid_zsr = [];
    for i = 1:length(err)
        toterr{j}.mean = toterr{j}.mean + err{i}.mean;
        toterr{j}.rmse = toterr{j}.rmse + err{i}.rmse;
        toterr{j}.nrmse = toterr{j}.nrmse + err{i}.nrmse;
        toterr{j}.euclid = [ toterr{j}.euclid ; err{i}.euclid ];   % stack errors at all time steps
        toterr{j}.euclid_zsr = [ toterr{j}.euclid_zsr ; sqrt( sum( results{i}.real.y.^2 , 2) ) ];   % euclidean error of a zeros state response
        toterr{j}.unscaled.euclid = [ toterr{j}.unscaled.euclid ; err{i}.unscaled.euclid ];   % stack errors at all time steps (unscaled units)
        toterr{j}.unscaled.euclid_zsr = [ toterr{j}.unscaled.euclid_zsr ; sqrt( sum( Ksysid.scaleup.y(results{i}.real.y).^2 , 2) ) ];   % euclidean error of a zeros state response (unscaled units)
    end
    toterr{j}.euclid_mean = sum( toterr{j}.euclid , 1 ) / length( toterr{j}.euclid );  % divide by total number of steps
    toterr{j}.euclid_zsr_mean = sum( toterr{j}.euclid_zsr , 1 ) / length( toterr{j}.euclid );  % divide by total number of steps
    toterr{j}.unscaled.euclid_mean = sum( toterr{j}.unscaled.euclid , 1 ) / length( toterr{j}.unscaled.euclid );  % divide by total number of steps
    toterr{j}.unscaled.euclid_zsr_mean = sum( toterr{j}.unscaled.euclid_zsr , 1 ) / length( toterr{j}.unscaled.euclid_zsr );  % divide by total number of steps
end

%% plot the accumulated errors of the systems

dim_linear = [];
err_linear = [];
dim_bilinear = [];
err_bilinear = [];
dim_nonlinear = [];
err_nonlinear = [];

% % normalized by zsr
% for i = 1 : length(toterr)
%     if strcmp( toterr{i}.model_type , 'linear' )
%         dim_linear = [ dim_linear , toterr{i}.dimK ];
%         err_linear = [ err_linear , toterr{i}.euclid_mean / toterr{i}.euclid_zsr_mean ];    % normalize by zsr error
%     elseif strcmp( toterr{i}.model_type , 'bilinear' )
%         dim_bilinear = [ dim_bilinear , toterr{i}.dimK ];
%         err_bilinear = [ err_bilinear , toterr{i}.euclid_mean / toterr{i}.euclid_zsr_mean ]; % normalize by zsr error
%     elseif strcmp( toterr{i}.model_type , 'nonlinear' )
%         dim_nonlinear = [ dim_nonlinear , toterr{i}.dimK ];
%         err_nonlinear = [ err_nonlinear , toterr{i}.euclid_mean / toterr{i}.euclid_zsr_mean ]; % normalize by zsr error
%     end  
% end

% Not normalized by zsr (and in unscaled units)
for i = 1 : length(toterr)
    if strcmp( toterr{i}.model_type , 'linear' )
        dim_linear = [ dim_linear , toterr{i}.dimK ];
        err_linear = [ err_linear , toterr{i}.unscaled.euclid_mean ]; 
    elseif strcmp( toterr{i}.model_type , 'bilinear' )
        dim_bilinear = [ dim_bilinear , toterr{i}.dimK ];
        err_bilinear = [ err_bilinear , toterr{i}.unscaled.euclid_mean ]; 
    elseif strcmp( toterr{i}.model_type , 'nonlinear' )
        dim_nonlinear = [ dim_nonlinear , toterr{i}.dimK ];
        err_nonlinear = [ err_nonlinear , toterr{i}.unscaled.euclid_mean ];
    end  
end


figure;
hold on;
plot( dim_linear , err_linear * 2.54 , '*-');  % multiply by 2.54 to convert from in to cm
plot( dim_bilinear , err_bilinear * 2.54 , '*-' ); % multiply by 2.54 to convert from in to cm
plot( dim_nonlinear , err_nonlinear * 2.54 , '*-' );   % multiply by 2.54 to convert from in to cm
hold off;
ylim([0,1]); % was [0,2]
set(gca, 'XScale', 'log')
xlim([10,1000]);
grid on; box on;
% xlabel('dim($\psi(\tilde{y},\tilde{u})$)' , 'Interpreter' , 'Latex');
% xlabel('Number of Basis Functions, i.e. dim($\psi(\tilde{y},\tilde{u})$)' , 'Interpreter' , 'Latex');
xlabel('Number of Basis Functions');
% ylabel('Average Model Prediction Error' , 'Interpreter' , 'Latex')
% ylabel('Average Error (Normalized)')
ylabel('Average Error (cm)')
legend({'Linear' , 'Bilinear' , 'Nonlinear'} , 'Location' , 'northwest');









