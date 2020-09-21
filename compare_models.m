% compare_models.m
%
% Simulate a bunch of models on the same set of validation data and compute
% overall error

% % load in the data file with validation trials in it
% [ datafile_name , datafile_path ] = uigetfile( 'datafiles/*.mat' , 'Choose data file for validation...' );
% data = load( [datafile_path , datafile_name] );

% load in the set of model files 
[ modelfile_name , modelfile_path ] = uigetfile( 'systems/*.mat' , 'Choose model files...' , 'MultiSelect' , 'on' );

% calculate error for each model
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
    for i = 1:length(err)
        toterr{j}.mean = toterr{j}.mean + err{i}.mean;
        toterr{j}.rmse = toterr{j}.rmse + err{i}.rmse;
        toterr{j}.nrmse = toterr{j}.nrmse + err{i}.nrmse;
        toterr{j}.euclid = [ toterr{j}.euclid ; err{i}.euclid ];   % stack errors at all time steps
    end
    toterr{j}.euclid_mean = sum( toterr{j}.euclid , 1 ) / length( toterr{j}.euclid );  % divide by total number of steps
end

%% plot the accumulated errors of the systems
dim_linear = [ toterr{1}.dimK , toterr{2}.dimK , toterr{3}.dimK , toterr{4}.dimK , toterr{5}.dimK , toterr{6}.dimK ];
err_linear = [ toterr{1}.euclid_mean , toterr{2}.euclid_mean , toterr{3}.euclid_mean , toterr{4}.euclid_mean , toterr{5}.euclid_mean , toterr{6}.euclid_mean ];
dim_bilinear = [ toterr{7}.dimK , toterr{8}.dimK , toterr{9}.dimK , toterr{10}.dimK ];
err_bilinear = [ toterr{7}.euclid_mean , toterr{8}.euclid_mean , toterr{9}.euclid_mean , toterr{10}.euclid_mean ];
dim_nonlinear = [ toterr{11}.dimK , toterr{12}.dimK , toterr{13}.dimK , toterr{14}.dimK ];
err_nonlinear = [ toterr{11}.euclid_mean , toterr{12}.euclid_mean , toterr{13}.euclid_mean , toterr{14}.euclid_mean ];

figure;
hold on;
plot( dim_linear , err_linear , '*-');
plot( dim_bilinear , err_bilinear , '*-' );
plot( dim_nonlinear , err_nonlinear , '*-' );
hold off;
ylim([0,2]);
set(gca, 'XScale', 'log')
xlim([10,1000]);
grid on; box on;
xlabel('dim($\psi(\tilde{y},\tilde{u})$)' , 'Interpreter' , 'Latex');
ylabel('Model Prediction Error' , 'Interpreter' , 'Latex')
legend({'Linear' , 'Bilinear' , 'Nonlinear'} , 'Location' , 'northwest');



