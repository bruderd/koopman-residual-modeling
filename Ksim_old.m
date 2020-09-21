classdef Ksim
    %mpcsim: Class for performing simulations of fake systems
    %   Build from a system class (like arm) and an mpc class
    
    properties
        sys;
        mpc;
        scaledown;  % for scaling data into range [-1 , 1]
        scaleup;    % for unscaling data from range [-1 , 1]
        ref;        % reference trajectory
        shape;      % reference point or trajectory of shape parameters (for arm system only)
    end
    
    methods
        function obj = Ksim( system_class , mpc_class , varargin )
            %CLASS CONSTRUCTOR
            %   Detailed explanation goes here
            
            obj.sys = system_class; % hold onto entire class
            obj.mpc = mpc_class;    % hold onto entire class
            
            % copy the scaling functions for easier access
            obj.scaledown = mpc_class.scaledown;
            obj.scaleup = mpc_class.scaleup;
%             obj = obj.get_refscale;   % get the scale matrix for reference traj.
            
            % set default values of optional inputs
            ref = [];
            shape = [];
            
            % replace default values with user input values
            obj = obj.parse_args( varargin{:} );
        end
        
        % parse_args: Parses the Name, Value pairs in varargin
        function obj = parse_args( obj , varargin )
            %parse_args: Parses the Name, Value pairs in varargin of the
            % constructor, and assigns property values
            for idx = 1:2:length(varargin)
                obj.(varargin{idx}) = varargin{idx+1} ;
            end
        end
        
        %% Define reference trajectories and state constraints
        
        % get_refscale (DO NOT USE): creates matrices for scaling the reference trajectory
        function obj = get_refscale( obj )
            % get_refscale: creates matrices for scaling the reference trajectory
            
            scaledown_vec = obj.mpc.projmtx * diag( obj.scaledown.z );     % gets scalefactor for each element of the reference trajectory
            scaleup_vec = obj.mpc.projmtx * diag( obj.scaleup.z );     % gets scalefactor for each element of the reference trajectory
            
            obj.scaledown.ref = diag( scaledown_vec );
            obj.scaleup.ref = diag( scaleup_vec );
        end
        
        %% Simulate the system with mpc controller
        
        % run_trial: Runs a simulation of system under mpc controller
        function results = run_trial( obj , ref , shape_bounds , x0 , u0)
            %run_trial: Runs a simulation of system under mpc controller.
            %   Tries to follow the trajectory in ref and impose the
            %   shape constraints in shape_bounds.
            %   Assume ref and shape_bounds have same sampling frequency as
            %   sytem, and that they are already scaled to be consistent 
            %   with the lifted model.
            %   ref - [ TotSteps , num_ref ], reference trajectory where
            %     each row is a desired point at the corresponding timestep
            %     (note that ref must be scaled same as model).
            %   shape_bounds - [ TotSteps (or 1) , 2*num_shapeParams ]
            %     min and max of shape parameters arranged in rows. First
            %     half of row is min values, 2nd half is max values.
            %   x0 - [1,n] initial condtion (state of underlying sys not output)
            %   u0 - [1,m] initial input
            
            % shorthand
            nd = obj.mpc.params.nd;
            Np = obj.mpc.horizon;
            
            % set value of shape_bounds and initial conditions if none provided
            if nargin < 3
                shape_bounds = [];
                x0 = zeros( nd+1 , obj.sys.params.nx );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            elseif nargin < 4
                x0 = zeros( nd+1 , obj.sys.params.nx );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            elseif nargin < 5
                x0 = kron( ones( nd+1 , 1 ) , x0 );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            else
                x0 = kron( ones( nd+1 , 1 ) , x0 );
                u0 = kron( ones( nd+1 , 1 ) , u0 );
            end
            
            % scale the reference trajectory
            ref_sc = ref;   % reference is already scaled
            
            % set initial condition (may add an input argument for this later)
            y0 = obj.sys.get_y( x0 );   % get corresponding outputs
%             y0 = kron( ones( size(x0,1) , 1 ) , obj.sys.get_shape_coeffs( x0(1,1:3) , 3 ) );    % NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!
            initial.y = y0; initial.u = u0;
            [ initial , zeta0 ] = obj.mpc.get_zeta( initial );    % LINE NOT NEEDED
            
            % initialize results struct
            results = struct;
            results.T = [ 0 ];
            results.U = [ u0( end , : ) ];
            results.Y = [ y0( end , : ) ];
            results.K = [ 0 ];
            results.R = [ ref(1,:) ];
            results.X = [ x0( end , : ) ];
            if isfield( obj.mpc.params , 'NLinput' )
                results.Z = [ obj.mpc.lift.zx( zeta0' )' ]; % lifted states
%                 results.UNL = [ zeros( 1 , obj.mpc.params.m ) ]; 
%                 results.E = [ zeros( 1 , obj.mpc.params.N ) ];
            else
                results.Z = [ obj.mpc.lift.econ_full( zeta0' )' ]; % lifted states
            end
            
            k = 1;
            while k < size( ref , 1 )
                
                % current time
                t = k * obj.mpc.params.Ts;
                
                % get current state and input with delays
                if k == 1
                    current.y = obj.scaledown.y( y0 );   
                    current.u = obj.scaledown.u( u0 );  
%                     current.y = y0;   % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%                     current.u = u0;   % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    if isfield( obj.mpc.params , 'NLinput' )    % replace with pseudoinput
%                         current.unl = results.E(end,:) * obj.mpc.model.Beta';
                    end
                elseif k < nd + 1
                    y = [ y0( k : end-1 , : ) ; results.Y ];
                    u = [ u0( k : end-1 , : ) ; results.U ];
                    current.y = obj.scaledown.y( y );
                    current.u = obj.scaledown.u( u ); 
%                     current.y = y;    % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%                     current.u = u;    % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    if isfield( obj.mpc.params , 'NLinput' )    % replace with pseudoinput
%                         current.unl = results.E(end,:) * obj.mpc.model.Beta';
                    end
                else
                    y = results.Y( end - nd : end , : );
                    u = results.U( end - nd : end , : );
                    current.y = obj.scaledown.y( y ); 
                    current.u = obj.scaledown.u( u ); 
%                     current.y = y;    % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%                     current.u = u;    % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    if isfield( obj.mpc.params , 'NLinput' )    % replace with pseudoinput
%                         current.unl = results.E(end,:) * obj.mpc.model.Beta';
                    end
                end
                
                % isolate the reference trajectory over the horizon
                if k + Np <= size( ref_sc , 1 )
                    refhor = ref_sc( k : k + Np , :);
                else
                    refhor = ref_sc( k : end , : );     % repeat last entry
                end
                    
                % isolate the shape_bounds over the horizon
                if ~isempty( shape_bounds )
                    len_sb = size( shape_bounds , 1);
                    if k + Np <= len_sb
                        shapehor = shape_bounds( k : k + Np , :);
                    elseif k <= len_sb
                        shapehor = [ shape_bounds( k : end , : ) ;...
                            kron( ones( Np + k - len_sb , 1 ) , shape_bounds(end,:) ) ]; % repeat last entry
                    else
                        shapehor = kron( ones( Np + 1, 1 ) , shape_bounds(end,:) ); % repeat last entry
                    end
                else
                    shapehor = [];
                end
                
                % get optimal input over horizon
                if isfield( obj.mpc.params , 'NLinput' )
                    [ E , z ] = obj.mpc.get_mpcInput_unl( current , refhor );
                    NU = E * obj.mpc.model.Beta';   % convert to nu
                    u_k_sc = obj.mpc.params.NLinput( [ NU(2,:) , current.y ]' )';
                else
                    [ U , z ] = obj.mpc.get_mpcInput( current , refhor );
                    
                    % if a solution was not found, break out of while loop
                    if any( isnan(U) )
                        break;
                    end
                    
                    % isolate input for this step (may need to make it U(1,:)
                    u_k_sc = U( 2 , : );
                end
                
%                 % compute the one step model error
%                 e = z - obj.mpc.model.A * results.Z(end,:)';
                
%                 % if the model is using a nonliner input, convert it
%                 if isfield( obj.mpc.params , 'NLinput' )
%                     unl = U(2,:);   % save the pseudoinput
%                     U = obj.mpc.params.NLinput( U' )';
%                 end
                
                % scaleup the input for the system simulation
                u_k = obj.scaleup.u( u_k_sc )';       % (no scaling) NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%                 u_k = u_k_sc';
                
                % simulate the system over one time-step
                x_k = results.X( end , : )';
                x_kp1 = obj.sys.simulate_Ts( x_k , u_k , [0 0]' );
                y_kp1 = obj.sys.get_y( x_kp1' );
%                 y_kp1 = obj.sys.get_shape_coeffs( x_kp1(1:3)' , 3 );    %NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                
                % record updated results
                results.T = [ results.T ; t ];
                results.U = [ results.U ; u_k' ];
                results.Y = [ results.Y ; y_kp1 ];
                results.K = [ results.K ; k ];
                results.R = [ results.R ; ref( k , : ) ];   % note that this is not scaled
                results.X = [ results.X ; x_kp1' ];
                results.Z = [ results.Z ; z'  ]; % current lifted state
                if isfield( obj.mpc.params , 'NLinput' )
%                     results.UNL = [ results.UNL ; unl ];    % pseudoinput
%                     results.E = [ results.E ; e' ];    % model error (with no input)
                end
                
                k = k + 1;  % increment step counter
            end
        end
        
        % run_trial_unl: Runs a simulation of system under mpc controller
        function results = run_trial_unl( obj , ref , shape_bounds , x0 , u0)
            %run_trial_unl: Runs a simulation of system under mpc controller.
            %   Tries to follow the trajectory in ref and impose the
            %   shape constraints in shape_bounds.
            %   Assume ref and shape_bounds have same sampling frequency as
            %   sytem, and that they are already scaled to be consistent 
            %   with the lifted model.
            %   ref - [ TotSteps , num_ref ], reference trajectory where
            %     each row is a desired point at the corresponding timestep
            %     (note that ref must be scaled same as model).
            %   shape_bounds - [ TotSteps (or 1) , 2*num_shapeParams ]
            %     min and max of shape parameters arranged in rows. First
            %     half of row is min values, 2nd half is max values.
            %   x0 - [1,n] initial condtion (state of underlying sys not output)
            %   u0 - [1,m] initial input
            
            % shorthand
            nd = obj.mpc.params.nd;
            Np = obj.mpc.horizon;
            
            % set value of shape_bounds and initial conditions if none provided
            if nargin < 3
                shape_bounds = [];
                x0 = zeros( nd+1 , obj.sys.params.nx );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            elseif nargin < 4
                x0 = zeros( nd+1 , obj.sys.params.nx );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            elseif nargin < 5
                x0 = kron( ones( nd+1 , 1 ) , x0 );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            else
                x0 = kron( ones( nd+1 , 1 ) , x0 );
                u0 = kron( ones( nd+1 , 1 ) , u0 );
            end
            
            % scale the reference trajectory
            ref_sc = ref;   % reference is already scaled
            
            % set initial condition (may add an input argument for this later)
            y0 = obj.sys.get_y( x0 );   % get corresponding outputs
            initial.y = y0; initial.u = u0;
            [ initial , zeta0 ] = obj.mpc.get_zeta( initial );    % LINE NOT NEEDED
            
            % initialize results struct
            results = struct;
            results.T = [ 0 ];
            results.U = [ u0( end , : ) ];
            results.Y = [ y0( end , : ) ];
            results.K = [ 0 ];
            results.R = [ ref(1,:) ];
            results.X = [ x0( end , : ) ];
            results.Z = [ obj.mpc.lift.zx( zeta0' )' ]; % lifted states
            results.E = [ zeros( 1 , obj.mpc.params.nzx ) ];
        
            k = 1;
            while k < size( ref , 1 )
                
                % current time
                t = k * obj.mpc.params.Ts;
                
                % get current state and input (no delays allowed)
                if k == 1   % this case may be extranneous
                    current.y = obj.scaledown.y( y0 );   
                    current.u = obj.scaledown.u( u0 );  
                    current.e = results.E(end,:);
                else
                    y = results.Y( end , : );
                    u = results.U( end , : );
                    current.y = obj.scaledown.y( y ); 
                    current.u = obj.scaledown.u( u ); 
                    current.e = results.E(end,:);
                end
                
                % isolate the reference trajectory over the horizon
                if k + Np <= size( ref_sc , 1 )
                    refhor = ref_sc( k : k + Np , :);
                else
                    refhor = ref_sc( k : end , : );     % repeat last entry
                end
                    
                % isolate the shape_bounds over the horizon
                if ~isempty( shape_bounds )
                    len_sb = size( shape_bounds , 1);
                    if k + Np <= len_sb
                        shapehor = shape_bounds( k : k + Np , :);
                    elseif k <= len_sb
                        shapehor = [ shape_bounds( k : end , : ) ;...
                            kron( ones( Np + k - len_sb , 1 ) , shape_bounds(end,:) ) ]; % repeat last entry
                    else
                        shapehor = kron( ones( Np + 1, 1 ) , shape_bounds(end,:) ); % repeat last entry
                    end
                else
                    shapehor = [];
                end
                
                % get optimal input over horizon
                [ E , z ] = obj.mpc.get_mpcInput_unl( current , refhor , shapehor );
                NU = E * obj.mpc.model.Beta';   % convert to nu
                u_k_sc = obj.mpc.params.NLinput( [ NU(2,:) , current.y ]' )';
                
                % scaleup the input for the system simulation
                u_k = obj.scaleup.u( u_k_sc )';
                
                % simulate the system over one time-step (use real system)
                x_k = results.X( end , : )';
                x_kp1 = obj.sys.simulate_Ts( x_k , u_k );
                y_kp1 = obj.sys.get_y( x_kp1' );
%                 y_kp1 = obj.sys.get_shape_coeffs( x_kp1(1:3)' , 3 );    %NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%                 % simulate the system over one time-step (use linear model and error as input) (USED to confirm MPC works as expected)
%                 z_kp1 = obj.mpc.model.A * results.Z(end,:)' + E(2,:)';
%                 y_kp1_sc = obj.mpc.model.C * z_kp1;
%                 y_kp1 = obj.scaleup.y( y_kp1_sc' );
%                 x_kp1 = [ 1 1 ]';
                

                % compute the one step model error
                y_kp1_sc = obj.scaledown.y( y_kp1 )';
                z_kp1 = obj.mpc.lift.zx( y_kp1_sc );
                e_kp1 = z_kp1 - obj.mpc.model.A * results.Z(end,:)';
                
                % record updated results
                results.T = [ results.T ; t ];
                results.U = [ results.U ; u_k' ];
                results.Y = [ results.Y ; y_kp1 ];
                results.K = [ results.K ; k ];
                results.R = [ results.R ; ref( k , : ) ];   % note that this is not scaled
                results.X = [ results.X ; x_kp1' ];
                results.Z = [ results.Z ; z_kp1'  ]; % current lifted state
%                 results.E = [ results.E ; e_kp1' ];    % model error (with no input)
                results.E = [ results.E ; E(2,:) ];    % full size pseudoinput
%                 results.E = [ results.E ; ( obj.mpc.model.B * obj.mpc.lift.zu( current.y' , u_k_sc' ) )' ];    % input part of model
                
                k = k + 1;  % increment step counter
            end
        end
        
        % run_trial_bilinear: Runs a simulation of system under mpc controller
        function results = run_trial_bilinear( obj , ref , shape_bounds , x0 , u0)
            %run_trial_unl: Runs a simulation of system under mpc controller.
            %   Tries to follow the trajectory in ref and impose the
            %   shape constraints in shape_bounds.
            %   Assume ref and shape_bounds have same sampling frequency as
            %   sytem, and that they are already scaled to be consistent 
            %   with the lifted model.
            %   ref - [ TotSteps , num_ref ], reference trajectory where
            %     each row is a desired point at the corresponding timestep
            %     (note that ref must be scaled same as model).
            %   shape_bounds - [ TotSteps (or 1) , 2*num_shapeParams ]
            %     min and max of shape parameters arranged in rows. First
            %     half of row is min values, 2nd half is max values.
            %   x0 - [1,n] initial condtion (state of underlying sys not output)
            %   u0 - [1,m] initial input
            
            % shorthand
            nd = obj.mpc.params.nd;
            Np = obj.mpc.horizon;
            
            % set value of shape_bounds and initial conditions if none provided
            if nargin < 3
                shape_bounds = [];
                x0 = zeros( nd+1 , obj.sys.params.nx );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            elseif nargin < 4
                x0 = zeros( nd+1 , obj.sys.params.nx );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            elseif nargin < 5
                x0 = kron( ones( nd+1 , 1 ) , x0 );
                u0 = zeros( nd+1 , obj.sys.params.nu );
            else
                x0 = kron( ones( nd+1 , 1 ) , x0 );
                u0 = kron( ones( nd+1 , 1 ) , u0 );
            end
            
            % scale the reference trajectory
            ref_sc = ref;   % reference is already scaled
            
            % set initial condition (may add an input argument for this later)
            y0 = obj.sys.get_y( x0 );   % get corresponding outputs
            initial.y = y0; initial.u = u0;
            [ initial , zeta0 ] = obj.mpc.get_zeta( initial );    % LINE NOT NEEDED
            
            % initialize results struct
            results = struct;
            results.T = [ 0 ];
            results.U = [ u0( end , : ) ];
            results.Y = [ y0( end , : ) ];
            results.K = [ 0 ];
            results.R = [ ref(1,:) ];
            results.X = [ x0( end , : ) ];
            results.Z = [ obj.mpc.lift.zx( zeta0' )' ]; % lifted states
            results.E = [ zeros( 1 , obj.mpc.params.m ) ];
            results.Ymodel = [ obj.scaledown.y( y0( end , : ) ) ];
            results.rankB = [ 0 ];
        
            k = 1;
            while k < size( ref , 1 )
                
                % current time
                t = k * obj.mpc.params.Ts;
                
                % get current state and input (no delays allowed)
                if k == 1   % this case may be extranneous
                    current.y = obj.scaledown.y( y0 );   
                    current.u = obj.scaledown.u( u0 );  
                    current.e = results.E(end,:);
                else
                    y = results.Y( end , : );
                    u = results.U( end , : );
                    current.y = obj.scaledown.y( y ); 
                    current.u = obj.scaledown.u( u ); 
                    current.e = results.E(end,:);
                end
                
                % isolate the reference trajectory over the horizon
                if k + Np <= size( ref_sc , 1 )
                    refhor = ref_sc( k : k + Np , :);
                else
                    refhor = ref_sc( k : end , : );     % repeat last entry
                end
                    
                % isolate the shape_bounds over the horizon
                if ~isempty( shape_bounds )
                    len_sb = size( shape_bounds , 1);
                    if k + Np <= len_sb
                        shapehor = shape_bounds( k : k + Np , :);
                    elseif k <= len_sb
                        shapehor = [ shape_bounds( k : end , : ) ;...
                            kron( ones( Np + k - len_sb , 1 ) , shape_bounds(end,:) ) ]; % repeat last entry
                    else
                        shapehor = kron( ones( Np + 1, 1 ) , shape_bounds(end,:) ); % repeat last entry
                    end
                else
                    shapehor = [];
                end
                
                % get optimal pseudoinput over horizon
                [ E , Zx ] = obj.mpc.get_mpcInput_bilinear( current , refhor , shapehor );
                e_kp1 = E(2,:)';     % isolate the pseudoinput for the next step
                u_k_sc = e_kp1;     % USE PSEUDOINPUT AS THE INPUT
                
%                 % DEBUG: what does the model think the state should given the pseudoinput?
%                 z_kp1_model = Zx(3,:)'; % what the model says the next lifted state should be
%                 y_kp1_model_sc = obj.mpc.model.C * z_kp1_model;

%                 % convert pseudoinput to actual input
%                 zx_stack = kron( eye(length(obj.mpc.params.u)) , Zx(2,:)' );
%                 zx0_stack = kron( eye(length(obj.mpc.params.u)) , Zx(1,:)' );
%                 Bzx = obj.mpc.model.B * zx_stack;
%                 Bzx0 = obj.mpc.model.B * zx0_stack;
% %                 Bzx0 = obj.mpc.cost.randB;  % DEBUG: use random B0 matrix
%                 u_k_sc = lsqminnorm( Bzx , Bzx0 * e_kp1 );
%                 rankB = rank( pinv(Bzx) * Bzx0 );   % DEBUG check rank of transformation
                
                % scaleup the input for the system simulation
                u_k = obj.scaleup.u( u_k_sc' )';
                
                % simulate the system over one time-step (use real system)
                x_k = results.X( end , : )';
                x_kp1 = obj.sys.simulate_Ts( x_k , u_k );
                y_kp1 = obj.sys.get_y( x_kp1' );
%                 y_kp1 = obj.sys.get_shape_coeffs( x_kp1(1:3)' , 3 );    %NEED TO UNDO THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%                 % simulate the system over one time-step (use linear model and error as input) (USED to confirm MPC works as expected)
%                 z_kp1 = obj.mpc.model.A * results.Z(end,:)' + E(2,:)';
%                 y_kp1_sc = obj.mpc.model.C * z_kp1;
%                 y_kp1 = obj.scaleup.y( y_kp1_sc' );
%                 x_kp1 = [ 1 1 ]';
                

                % compute the one step model error
                y_kp1_sc = obj.scaledown.y( y_kp1 )';
                z_kp1 = obj.mpc.lift.zx( y_kp1_sc );
                e_kp1 = z_kp1 - obj.mpc.model.A * results.Z(end,:)';
                
                % record updated results
                results.T = [ results.T ; t ];
                results.U = [ results.U ; u_k' ];
                results.Y = [ results.Y ; y_kp1 ];
                results.K = [ results.K ; k ];
                results.R = [ results.R ; ref( k , : ) ];   % note that this is not scaled
                results.X = [ results.X ; x_kp1' ];
                results.Z = [ results.Z ; z_kp1'  ]; % current lifted state
                results.E = [ results.E ; E(2,:) ];    % full size pseudoinput
%                 results.Ymodel = [ results.Ymodel ; y_kp1_model_sc' ];
%                 results.rankB = [ results.rankB ; rankB ];  % DEBUG
                
                k = k + 1;  % increment step counter
            end
        end
    end
end