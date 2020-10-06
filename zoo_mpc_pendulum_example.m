% zoo_mpc_pendulum_example
%
% Use residual Koopman model to make a "real" pendulum behave the same as a
% template pendulum model (Only difference is that one of them has a
% heavier mass at the end)
%
% Assumes that Ksysid, Arm_real, and Arm_temp classes are all in the
% workspace

%% Generate mpc class

% (Assumes that Ksysid class is already in the workspace)
Kmpc = Kmpc( Ksysid ,...
        'horizon' , 2 ,...
        'input_bounds' , [-pi,pi],... %[ -7*pi/8 , 7*pi/8 ] ,... % DO NOT USE INF
        'input_slopeConst' , [1e-3],... %1e-1 ,...
        'input_smoothConst' , [],... %[1e-1] ,...
        'state_bounds' , [] ,...
        'cost_running' , 10 ,...   % 0.1
        'cost_terminal' , 100 ,...  % 100
        'cost_input' , 0 ,...    % 1e-1
        'projmtx' , Ksysid.model.C(1:2,:) );  % just error state

%% set parameters
saveon = true;  % save a datafile if this is true
num_steps = 100; % number of steps to take in trial
u_min = -pi;    % minimum input
u_max = pi;     % maximum input

%% Generate sequence of inputs for the template model

% % random
% rand_vec_utemp = rand( num_steps , 1 );
% u_temp = (1 - rand_vec_utemp) * u_min + rand_vec_utemp * u_max;

% ramp
u_temp = [linspace( 0 , 7*pi/8 , num_steps )' ,...
          linspace( 0 , -4*pi/8 , num_steps )' ];

% % step
% u_temp = ones( num_steps , Ksysid.params.m ) * pi/2;

% % steps
% u_temp = [ ones( num_steps - floor(num_steps/2) , 1 ) * pi/2;...
%            -ones( floor(num_steps/2) , 1 ) * pi/4];

%% Simulate the systems using the residual model mpc controller

% set initial conditions
Alpha0_real = zeros( Arm_real.params.nx , 1 );    % joint angle/velocity
Alpha0_temp = zeros( Arm_temp.params.nx , 1 );    % joint anlge/velocity
x0_real = Arm_real.get_y( Alpha0_real' )';   % end effector coordinates
x0_temp = Arm_temp.get_y( Alpha0_temp' )';   % end effector coordinates

% preallocate
u_res = zeros( num_steps , Arm_real.params.nu );    % residual model input
u_real = zeros( num_steps , Arm_real.params.nu );
x_real = zeros( num_steps , Arm_real.params.ny );
x_temp = zeros( num_steps , Arm_temp.params.ny );
x_comp = zeros( num_steps , Arm_real.params.ny );
Alpha_real = zeros( num_steps , Arm_real.params.nx );
Alpha_temp = zeros( num_steps , Arm_temp.params.nx );
Alpha_comp = zeros( num_steps , Arm_temp.params.nx );
u_real(1,:) = u_temp(1,:);  % first input is same as template model
x_real(1,:) = x0_real';
x_temp(1,:) = x0_temp';
x_comp(1,:) = x0_real';
Alpha_real(1,:) = Alpha0_real';
Alpha_temp(1,:) = Alpha0_temp';
Alpha_comp(1,:) = Alpha0_real';
for i = 1 : num_steps - 1
    % lift the state of the residual model
    xres_k = [ x_real(i,:) - x_temp(i,:) , x_real(i,:) , u_temp(i,:) ]';
    ures_k = ( u_real(i,:) - u_temp(i,:) )';
    
    % solve for optimal input into residual model using mpc
    if i == 1
        current.y = Ksysid.scaledown.y( xres_k' );
        current.u = Ksysid.scaledown.u( zeros( 1 , Ksysid.params.m ) );
    else
        current.y = Ksysid.scaledown.y( xres_k' );
        current.u = Ksysid.scaledown.u( ures_k' );
    end
    refhor = [ 0 , 0 ; 0 , 0]; % goal is to drive error to zero
    if strcmp( Kmpc.model_type , 'linear' )
        [ U , z ] = Kmpc.get_mpcInput( current , refhor );
    elseif strcmp( Kmpc.model_type , 'bilinear' )
%         [ U , z ] = obj.mpc.get_mpcInput_bilinear( current , refhor );
        [ U , z ] = Kmpc.get_mpcInput_bilinear_iter( current , refhor , 1 );
    elseif strcmp( Kmpc.model_type , 'nonlinear' )
        [ U , z ] = Kmpc.get_mpcInput_nonlinear( current , refhor );
    end
    % scaleup the input
    u_res(i+1,:) = Ksysid.scaleup.u( U(end,:) );
    u_real(i+1,:) = u_res(i+1,:) + u_temp(i+1,:);
    
    % simulate the template model k --> k+1
    Alpha_temp(i+1,:) = Arm_temp.simulate_Ts( Alpha_temp(i,:)' , u_temp(i,:)' , [] , Ksysid.params.Ts )';
    x_temp(i+1,:) = Arm_temp.get_y( Alpha_temp(i+1,:) );
    
    % simulate the real model k --> k+1
    Alpha_real(i+1,:) = Arm_real.simulate_Ts( Alpha_real(i,:)' , u_real(i,:)' , [] , Ksysid.params.Ts )';
    x_real(i+1,:) = Arm_real.get_y( Alpha_real(i+1,:) );
    
    % For comparison, simulate the real system under same inputs as temp
    Alpha_comp(i+1,:) = Arm_real.simulate_Ts( Alpha_comp(i,:)' , u_temp(i,:)' , [] , Ksysid.params.Ts )';
    x_comp(i+1,:) = Arm_real.get_y( Alpha_comp(i+1,:) );
end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    