% compare_mpc_thesis
%   Simulates a linear, bilinear, and nonlinear mpc controller for a
%   trajectory following task.
%
%   This is not a general purpose function. It specifically compares the
%   controllers described in Chapter 4 of the thesis.

%% Choose whether to save siulation results or not
saveon = false;

%% Load the models

% load the Arm system class
temp_arm = load('C:\Users\danie\OneDrive\Documents\MATLAB\UM-Matlab\Koopman\ver14-loads-4-thesis\systems\thesis-arm-markers_noload_3-mods_1-links_20hz\thesis-arm-markers_noload_3-mods_1-links_20hz.mat');
Arm = temp_arm.Arm;

% load the model classes
temp_lin = load('C:\Users\danie\OneDrive\Documents\MATLAB\UM-Matlab\Koopman\ver14-loads-4-thesis\systems\thesis-arm-markers_noload_3-mods_1-links_20hz\models\linear_poly-3_n-6_m-3_del-0_2020-06-09_16-42.mat');
temp_bilin = load('C:\Users\danie\OneDrive\Documents\MATLAB\UM-Matlab\Koopman\ver14-loads-4-thesis\systems\thesis-arm-markers_noload_3-mods_1-links_20hz\models\bilinear_poly-3_n-6_m-3_del-0_2020-06-09_16-43.mat');
temp_nonlin = load('C:\Users\danie\OneDrive\Documents\MATLAB\UM-Matlab\Koopman\ver14-loads-4-thesis\systems\thesis-arm-markers_noload_3-mods_1-links_20hz\models\nonlinear_poly-3_n-6_m-3_del-0_2020-06-13_14-10.mat');
Ksysid_lin = temp_lin.sysid_class;  % linear model
Ksysid_bilin = temp_bilin.sysid_class;  % bilinear model
Ksysid_nonlin = temp_nonlin.sysid_class;    % nonlinear model

%% Construct an mpc controller from each model

% linear mpc controller
Kmpc_lin = Kmpc( Ksysid_lin ,...
        'horizon' , 10 ,...
        'input_bounds' , [],... % DO NOT USE INF
        'input_slopeConst' , [1e-1],... %1e-1 ,...
        'input_smoothConst' , [],... %[1e-1] ,...
        'state_bounds' , [] ,...
        'cost_running' , 10 ,...   % 0.1
        'cost_terminal' , 100 ,...  % 100
        'cost_input' , 0.1 * [ 3e-2 , 2e-2 , 1e-2 ]' ,...    % 1e-1
        'projmtx' , Ksysid_lin.model.C(end-1:end,:) );% ,...  % just end effector
Ksim_lin = Ksim( Arm , Kmpc_lin );

% bilinear mpc controller
Kmpc_bilin = Kmpc( Ksysid_bilin ,...
        'horizon' , 10 ,...
        'input_bounds' , [],... % DO NOT USE INF
        'input_slopeConst' , [1e-1],... %1e-1 ,...
        'input_smoothConst' , [],... %[1e-1] ,...
        'state_bounds' , [] ,...
        'cost_running' , 10 ,...   % 0.1
        'cost_terminal' , 100 ,...  % 100
        'cost_input' , 0.1 * [ 3e-2 , 2e-2 , 1e-2 ]' ,...    % 1e-1
        'projmtx' , Ksysid_bilin.model.C(end-1:end,:) );% ,...  % just end effector
Ksim_bilin = Ksim( Arm , Kmpc_bilin );
    
Kmpc_nonlin = Kmpc( Ksysid_nonlin ,...
        'horizon' , 10 ,...
        'input_bounds' , [],... % DO NOT USE INF
        'input_slopeConst' , [1e-1],... %1e-1 ,...
        'input_smoothConst' , [],... %[1e-1] ,...
        'state_bounds' , [] ,...
        'cost_running' , 10 ,...   % 0.1
        'cost_terminal' , 100 ,...  % 100
        'cost_input' , 0.1 * [ 3e-2 , 2e-2 , 1e-2 ]' ,...    % 1e-1
        'projmtx' , Ksysid_nonlin.model.C(end-1:end,:) );% ,...  % just end effector
Ksim_nonlin = Ksim( Arm , Kmpc_nonlin );

%% Load in reference trajectory

% Michigan block M
temp_ref = load('C:\Users\danie\OneDrive\Documents\MATLAB\UM-Matlab\Koopman\ver14-loads-4-thesis\trajectories\files\blockM_c0p45-0p35_0p5x0p5_15sec.mat');
ref = temp_ref.ref;
  
%% Sumulate each controller performing task

res_lin = Ksim_lin.run_trial_mpc( ref.y , [0 0 0 0 0 0] , [0 0 0] , [ 0 , 0 ]);
res_bilin = Ksim_bilin.run_trial_mpc( ref.y , [0 0 0 0 0 0] , [0 0 0] , [ 0 , 0 ]);
res_nonlin = Ksim_nonlin.run_trial_mpc( ref.y , [0 0 0 0 0 0] , [0 0 0] , [ 0 , 0 ]);

%% save results (OPTIONAL)
if saveon
    sim_folder = [ 'systems' , filesep , Kmpc_lin.model.params.sysParams.sysName , filesep , 'simulations' ];
    mkdir( sim_folder , ref.name );
    filepath = [ sim_folder , filesep , ref.name ];
    
    filename_lin = [ filepath , filesep , Kmpc_lin.params.classname , '.mat' ];
    save( filename_lin , 'res_lin' );
    
    filename_bilin = [ filepath , filesep , Kmpc_bilin.params.classname , '.mat' ];
    save( filename_bilin , 'res_bilin' );
    
    filename_nonlin = [ filepath , filesep , Kmpc_nonlin.params.classname , '.mat' ];
    save( filename_nonlin , 'res_nonlin' );
end

%% Compute error

err4barplot = [ mean( res_lin.err ) , 0 , 0 , mean( res_bilin.err ) , 0 , 0 , mean( res_nonlin.err ) , 0 ]' * 100; % cm
comp4barplot = [ 0 , mean( res_lin.comp_time) , 0 , 0 , mean( res_bilin.comp_time) , 0 , 0 , mean( res_nonlin.comp_time) ]' * 1000; % ms
data4barplot = [ mean( res_lin.err ) , mean( res_lin.comp_time ) ;...
                mean( res_bilin.err ) , mean( res_bilin.comp_time ) ;...
                mean( res_nonlin.err ) , mean( res_nonlin.comp_time ) ];


% bar graph (mean errror and computation time)
figure;
hold on;
yyaxis left;    % left axis: error
ylabel('Mean Tracking Error (cm)')
bar(err4barplot);
set(gca, 'YScale', 'log')
ylim([1e-1,1e3])
text( 1:8 , err4barplot , num2str(err4barplot,3) , 'vert','bottom','horiz','center'); % bar labels
yyaxis right;   % right axis: computation time
ylabel('Mean Computation Time (ms)')
bar(comp4barplot);
yline( Ksysid_lin.params.Ts*1000 , '--');   % show sampling time in ms
set(gca, 'YScale', 'log')
ylim([1e0,1e4])
xticks([1.5,4.5,7.5]);
xticklabels({'Linear' , 'Bilinear' , 'Nonlinear'});
text( 1:8 , comp4barplot , num2str(comp4barplot,3) , 'vert','bottom','horiz','center');   % bar labels
hold off;
box on;

%% Plot results

colormap lines;
cmap = colormap;

figure;

subplot(1,3,1); % linear controller results
hold on;
plot( 100*res_lin.R(:,1) , 100*res_lin.R(:,2) , 'Color' , [0 0 0 0.5] , 'LineWidth' , 2 );
plot( 100*res_lin.Y(:,end-1) , 100*res_lin.Y(:,end) , 'Color' , cmap(1,:) , 'LineWidth' , 2);
hold off;
xlim([-100,100]);
ylim([-100,100]);
grid on;
box on;
ylabel('$\hat{\beta}$ (cm)' , 'Interpreter' , 'Latex');
xlabel('$\hat{\alpha}$ (cm)' , 'Interpreter' , 'Latex');
set(gca, 'YDir','reverse')
daspect([1 1 1]);   % make axis ratio 1:1
title('Linear');

subplot(1,3,2); % bilinear controller results
hold on;
plot( 100*res_lin.R(:,1) , 100*res_lin.R(:,2) , 'Color' , [0 0 0 0.5] , 'LineWidth' , 2 );
plot( 100*res_bilin.Y(:,end-1) , 100*res_bilin.Y(:,end) , 'Color' , cmap(2,:) , 'LineWidth' , 2);
hold off;
xlim([0,100]);
ylim([0,100]);
grid on;
box on;
xlabel('$\hat{\alpha}$ (cm)' , 'Interpreter' , 'Latex');
set(gca, 'YDir','reverse')
daspect([1 1 1]);   % make axis ratio 1:1
title('Bilinear');

subplot(1,3,3); % linear controller results
hold on;
plot( 100*res_lin.R(:,1) , 100*res_lin.R(:,2) , 'Color' , [0 0 0 0.5] , 'LineWidth' , 2 );
plot( 100*res_nonlin.Y(:,end-1) , 100*res_nonlin.Y(:,end) , 'Color' , cmap(3,:) , 'LineWidth' , 2);
hold off;
xlim([0,100]);
ylim([0,100]);
grid on;
box on;
xlabel('$\hat{\alpha}$ (cm)' , 'Interpreter' , 'Latex');
set(gca, 'YDir','reverse')
daspect([1 1 1]);   % make axis ratio 1:1
title('Nonlinear');
    
    






    