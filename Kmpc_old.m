classdef Kmpc
    %Kmpc: Model predictive controller class
    %   Detailed explanation goes here
    
    properties
        params; % paramaters of the system
        model;  % linear model of the system
        lift;   % lifting functions for system
        basis;  % symbolic basis set of observables
        horizon;
        input_bounds;
        input_slopeConst;
        input_smoothConst;
        state_bounds;
        cost_running;
        cost_terminal;
        cost_input;
        bilinear;   % true/false, is this a bilinear model?
        projmtx; % projection matrix from liftes state (z) to reference state   
        cost;   % stores cost matrices
        constraints;    % stores constraint matrices
        set_constRHS;  % function that sets the value of the RHS of constraints
        get_zeta;   % function that constructs zeta from state and input data in time
        
        scaledown;  % functions for scaling to [-1,1]
        scaleup;    % functions for scaling from [-1,1]
    end
    
    methods
        % CLASS CONSTRUCTOR
        function obj = Kmpc_old( sysid_class , varargin )
            %mpc: Construct an instance of this class
            %   sysid_class - sysid class object with a model and params
            %    properties
            %   varargin - Name, Value pairs for class properties
            
            % take some properties/methods from the sysid class
            obj.params = sysid_class.params;
            obj.model = sysid_class.model;
            obj.lift = sysid_class.lift;
            obj.basis = sysid_class.basis;
            obj.get_zeta = @sysid_class.get_zeta;   % copies this method for convenience
            obj.scaledown = sysid_class.scaledown;
            obj.scaleup = sysid_class.scaleup;
            
            % define default values of properties
            obj.horizon = floor( 1 / obj.params.Ts );
            obj.input_bounds = [];  % [min,max] can be 1x2 or mx2
            obj.input_slopeConst = [];
            obj.input_smoothConst = [];
            obj.state_bounds = []; % [min,max] can be 1x2 or nx2 
            obj.cost_running = 0.1;
            obj.cost_terminal = 100;
            obj.cost_input = 0;
            obj.projmtx = obj.model.C;   % recovers measured state (could also use Cshape)
            obj.cost = [];
            obj.constraints = [];
            obj.bilinear = false;   % default is not a bilinear system
            
            % replace default values with user input values
            obj = obj.parse_args( varargin{:} );
            
            % if the system has nonlinear input, change the dimension of input
            if isfield( obj.params , 'NLinput' ) && ~obj.bilinear
%                 obj.params.m = obj.params.mnu;
%                 obj.model.params.m = obj.params.mnu;
                obj.params.m = obj.params.nzx;  % pseudoinput is e (error term)
                obj.model.params.m = obj.params.nzx;
            end
            
            % resize some properties if they aren't full size already
            obj = obj.expand_props;
            
            % get cost and constraint matrices
            if isfield( obj.params , 'NLinput' ) && ~obj.bilinear
                obj = obj.get_costMatrices_unl;
                obj = obj.get_constraintMatrices_unl;
            elseif strcmp( sysid_class.model_type , 'bilinear' )
                obj = obj.get_costMatrices_bilinear;
                obj = obj.get_constraintMatrices_bilinear;
            else
                obj = obj.get_costMatrices; 
                obj = obj.get_constraintMatrices;
            end
            
        end
        
        % parse_args: Parses the Name, Value pairs in varargin
        function obj = parse_args( obj , varargin )
            %parse_args: Parses the Name, Value pairs in varargin of the
            % constructor, and assigns property values
            for idx = 1:2:length(varargin)
                obj.(varargin{idx}) = varargin{idx+1} ;
            end
        end
        
        % expand_props: Converts props from shorthand to fully defined
        function obj = expand_props( obj )
            %expand_props: Converts props from shorthand to fully defined
            %   e.g. input_bounds = [ -Inf , Inf ] but params.m = 3,
            %   ==> [ -Inf , Inf ; -Inf , Inf ; -Inf , Inf ]
            
            % input_bounds
            if ~isempty( obj.input_bounds ) && size( obj.input_bounds , 1 ) ~= obj.params.m
                obj.input_bounds = kron( ones( obj.params.m , 1 ) , obj.input_bounds );
            end
            
            % state_bounds
            if ~isempty( obj.state_bounds ) && size( obj.state_bounds , 1 ) ~= obj.params.n
                obj.state_bounds = kron( ones( obj.params.n , 1 ) , obj.state_bounds );
            end     
        end
        
        %% linear MPC functions
        
        % get_costMatrices: Contructs the matrices for the mpc optim. problem
        function obj = get_costMatrices( obj )
            %get_costMatrices: Constructs cost the matrices for the mpc 
            % optimization problem.
            %   obj.cost has fields H, G, D, A, B, C, Q, R
            
            % define cost function matrices
            % Cost function is defined: U'HU + ( z0'G + Yr'D )U
            
            model = obj.model;
            
            % A
            N = size(model.A,1);
            A = sparse( N*(obj.horizon+1) , N );
            for i = 0 : obj.horizon
                A( (N*i + 1) : N*(i+1) , : ) = model.A^i ;
            end
            
            % B
            Bheight = N*(obj.horizon+1);
            Bcolwidth = size(model.B,2);
            Bcol = sparse( Bheight , Bcolwidth );    % first column of B matrix
            for i = 1 : obj.horizon
                Bcol( (N*i + 1) : N*(i+1) , : ) = model.A^(i-1) * model.B ;
            end
            
            Lshift = spdiags( ones( N*obj.horizon , 1 ) , -N , N*(obj.horizon+1) , N*(obj.horizon+1) );    % lower shift operator
            
            Bwidth = size(model.B,2)*(obj.horizon);    % total columns in B matrix
            Bblkwidth = obj.horizon;   % how many Bcol blocks wide B matrix is
            B = spalloc( Bheight , Bwidth , floor(Bheight * Bwidth / 2) ); % initialze sparse B matrix
            B(: , 1:Bcolwidth) = Bcol;
            for i = 2 : Bblkwidth
                B(: , (i-1)*Bcolwidth+1 : i*Bcolwidth) = Lshift * B(: , (i-2)*Bcolwidth+1 : (i-1)*Bcolwidth);
            end
            
            % C: matrix that projects lifted state into reference trajectory space
            C = kron( speye(obj.horizon+1) , obj.projmtx);
            nproj = size( obj.projmtx , 1 );
            
            % Q: Error magnitude penalty
            Q = kron( speye(obj.horizon+1) , eye(nproj) * obj.cost_running); % error magnitude penalty (running cost) (default 0.1)
            Q(end-nproj+1 : end , end-nproj+1 : end) = eye(nproj) * obj.cost_terminal;    % (terminal cost) (default 100)
            
            % R: Input magnitude penalty
            R = kron( speye(obj.horizon) , eye(model.params.m) * obj.cost_input );  % input magnitude penalty (for flaccy use 0.5e-2) (new videos used 0.5e-3)

            % H, G, D
            H = B' * C' * Q * C * B + R;
            G = 2 * A' * C' * Q * C * B;
            D = -2 * Q * C * B;
            
            % set outputs
            obj.cost.H = H; obj.cost.G = G; obj.cost.D = D; % constructed matrices
            obj.cost.A = A; obj.cost.B = B; obj.cost.C = C; obj.cost.Q = Q; obj.cost.R = R; % component matrices
        end
        
        % get_constraintMatrices: Constructs the constraint matrices
        function obj = get_constraintMatrices( obj )
            %get_constraintMatrices: Constructs the constraint matrices for
            % the mpc optimization problem.
            %   obj.constraints has fields L, M, F, E, (c?)
            %   F is for input constraints
            %   E is for state constraints
            
            % shorten some variable names
            Np = obj.horizon;     % steps in horizon
            params = obj.params;    % linear system model parameters
            cost = obj.cost;     % cost matrices
            
            F = []; E = [];     % initialize empty matrices
            c = [];      % RHS
            
            % input_bounds
            if ~isempty( obj.input_bounds )
                num = 2*params.m;     % number of input bound constraints
                
                % F: input_bounds
                Fbounds_i = [ -speye(params.m) ; speye(params.m) ];    % diagonal element of F, for bounded inputs
                Fbounds = sparse( num * (Np+1) , size(cost.B,2) );  % no constraints, all zeros
                Fbounds( 1:num*Np , 1:Np*params.m ) = kron( speye(Np) , Fbounds_i );     % fill in nonzeros
                F = [ F ; Fbounds ];    % append matrix
                
                % E: input_bounds (just zeros)
                Ebounds = sparse( num * (Np+1) , size(cost.B,1) );  % no constraints, all zeros
                E = [ E ; Ebounds ];    % append matrix
                
                % c: input_bounds
                if isfield( obj.params , 'NLinput' )    % don't scale input if it's nonlinear
                    input_bounds_sc = obj.input_bounds;
                else
                    input_bounds_sc = obj.scaledown.u( obj.input_bounds' )';   % scaled down the input bounds
                end
                cbounds_i = [ -input_bounds_sc(:,1); input_bounds_sc(:,2) ]; % [ -umin ; umax ]
                cbounds = zeros( num * (Np+1) , 1);    % initialization
                cbounds(1 : num*Np) = kron( ones( Np , 1 ) , cbounds_i );     % fill in nonzeros
                c = [ c ; cbounds ];    % append vector
            end
            
            % input_slopeConst
            if ~isempty( obj.input_slopeConst )
                % F: input_slopeConst
                Fslope_i = speye(params.m);
                Fslope_neg = [ kron( speye(Np-1) , -Fslope_i ) , sparse( params.m * (Np-1) , params.m ) ];
                Fslope_pos = [ sparse( params.m * (Np-1) , params.m ) , kron( speye(Np-1) , Fslope_i ) ];
                Fslope_top = Fslope_neg + Fslope_pos;
                Fslope = [ Fslope_top ; -Fslope_top];
                F = [ F ; Fslope ];     % append matrix

                % E: input_slopeConst (just zeros)
                E = [ E ; sparse( 2 * params.m * (Np-1) , size(cost.B,1) ) ];
                
                % c: input_slopeConst
                if isfield( obj.params , 'NLinput' )    % don't scale slope if it's nonlinear
                    slope_lim = obj.input_slopeConst;
                else
                    slope_lim = obj.input_slopeConst * mean( params.scale.u_factor );  % scale down the 2nd deriv. limit
                end
                cslope_top = slope_lim * ones( params.m * (Np-1) , 1 );
                cslope = [ cslope_top ; cslope_top ];
                c = [ c ; cslope ];     % append vector
            end
            
            % input_smoothConst
            if ~isempty( obj.input_smoothConst )
                % F: input_smoothConst
                Fsmooth_i = speye(params.m);
                Fsmooth_lI = [ kron( speye(Np-2) , Fsmooth_i ) , sparse( params.m * (Np-2) , 2 * params.m ) ];
                Fsmooth_2I = [ sparse( params.m * (Np-2) , params.m ) , kron( speye(Np-2) , -2*Fslope_i ) , sparse( params.m * (Np-2) , params.m ) ];
                Fsmooth_rI = [ sparse( params.m * (Np-2) , 2 * params.m ) , kron( speye(Np-2) , Fslope_i ) ];
                Fsmooth_top = Fsmooth_lI + Fsmooth_2I + Fsmooth_rI;
                Fsmooth = [ Fsmooth_top ; -Fsmooth_top ];
                F = [ F ; Fsmooth ];
                
                % E: input_smoothConst
                E = [ E ; sparse( 2 * params.m * (Np-2) , size(cost.B,1) ) ];
                
                % c: input_smoothConst
                smooth_lim = params.Ts^2 * obj.input_smoothConst * mean( params.scale.u_factor );  % scale down the 2nd deriv. limit
                csmooth = smooth_lim * ones( size(Fsmooth,1) ,1);
                c = [ c ; csmooth ];
            end
            
            % state_bounds
            if ~isempty( obj.state_bounds )
                num = 2*params.n;   % number of state bound constraints
                
                % E: state_bounds
                Esbounds_i = [ -speye(params.n) ; speye(params.n) ];    % diagonal element of E, for bounding low dim. states (first n elements of lifted state)
                Esbounds = sparse( num * (Np+1) , size(cost.A,1) );  % no constraints, all zeros
                Esbounds( 1:num*(Np+1) , 1:(Np+1)*params.n ) = kron( speye(Np+1) , Esbounds_i );     % fill in nonzeros
                E = [ E ; Esbounds ];    % append matrix
                
                % F: state_bounds (all zeros)
                Fsbounds = zeros( size( Esbounds , 1 ) , size( cost.B , 2 ) );
                F = [ F ; Fsbounds ];    % append matrix
                
                % c: state_bounds
                state_bounds_sc = obj.scaledown.y( obj.state_bounds' )';    % scaled down state bounds
                csbounds_i = [ -state_bounds_sc(:,1) ; state_bounds_sc(:,2) ]; % [ -ymin ; ymax ]
                csbounds = kron( ones( Np+1 , 1 ) , csbounds_i );     % fill in nonzeros
                c = [ c ; csbounds ];    % append vector
            end
            
            % set outputs
            obj.constraints.F = F;
            obj.constraints.E = E;    
            obj.constraints.c = c;
            obj.constraints.L = F + E * cost.B;
            obj.constraints.M = E * cost.A;
        end
        
        % get_mpcInput: Solve the mpc problem to get the input over entire horizon
        function [ U , z ]= get_mpcInput( obj , traj , ref )
            %get_mpcInput: Soves the mpc problem to get the input over
            % entire horizon.
            %   traj - struct with fields y , u. Contains the measured
            %    states and inputs for the past ndelay+1 timesteps.
            %   ref - matrix containing the reference trajectory for the
            %    system over the horizon (one row per timestep).
            %   shape_bounds - [min_shape_parameters , max_shape_parameters] 
            %    This is only requred if system has shape constraints 
            %   (note: size is num of shape observables x 2)
            %   z - the lifted state at the current timestep
            
            % shorthand variable names
            Np = obj.horizon;       % steps in the horizon
            nd = obj.params.nd;     % number of delays
            
            % construct the current value of zeta
            [ ~ , zeta_temp ] = obj.get_zeta( traj );
            zeta = zeta_temp( end , : )';   % want most recent points
            
            % lift zeta
            z = obj.lift.econ_full( zeta );
            
            % check that reference trajectory has correct dimensions
            if size( ref , 2 ) ~= size( obj.projmtx , 1 )
                error('Reference trajectory is not the correct dimension');
            elseif size( ref , 1 ) > Np + 1
                ref = ref( 1 : Np + 1 , : );    % remove points over horizon
            elseif size( ref , 1 ) < Np + 1
                ref_temp = kron( ones( Np+1 , 1 ) , ref(end,:) );
                ref_temp( 1 : size(ref,1) , : ) = ref;
                ref = ref_temp;     % repeat last point for remainer of horizon
            end
            
            % vectorize the reference trajectory
            Yr = reshape( ref' , [ ( Np + 1 ) * size(ref,2) , 1 ] );
            
            % get the RHS of the inequality constraints
            c = obj.constraints.c;
            
            % setup matrices for gurobi solver
            H = obj.cost.H;      % removed factor of 2 on 12/10/2018
            f = ( z' * obj.cost.G + Yr' * obj.cost.D )';
            A = obj.constraints.L;
            b = - obj.constraints.M * z + c;
            
            % tack on "memory" constraint to fix initial input u_0
            Atack = [ [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - obj.params.m ) ];
%             Atack_bot = [ sparse( 2*obj.params.m , obj.params.m) , [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - 2*obj.params.m ) ];
%             Atack = [ Atack_top ; Atack_bot ];
            if isfield( traj , 'unl' )
                btack = [ traj.unl(end,:)' ; -traj.unl(end,:)' ];
            else
                btack = [ traj.u(end,:)' ; -traj.u(end,:)' ];
            end
            A = [A ; Atack];    % tack on memory constraint
            b = [b ; btack];
            
            % solve the MPC problem
            Uvec = quadprog_gurobi( 2*H , f , A , b );   % solve using gurobi (returns NaNs of cannot be solved)
            % U = quadprog( 2*H , f , A , b );     % solve using matlab
            
            % reshape the output so each input will have one row (first row equals current input)
            U = reshape( Uvec , [ obj.params.m , Np ] )';
        end
        
        % get_mpcInput_noIC: Solve the mpc problem to get the input over entire horizon
        function [ U , z ]= get_mpcInput_noIC( obj , traj , ref )
            %get_mpcInput: Soves the mpc problem to get the input over
            % entire horizon.
            %   traj - struct with fields y , u. Contains the measured
            %    states and inputs for the past ndelay+1 timesteps.
            %   ref - matrix containing the reference trajectory for the
            %    system over the horizon (one row per timestep).
            %   shape_bounds - [min_shape_parameters , max_shape_parameters] 
            %    This is only requred if system has shape constraints 
            %   (note: size is num of shape observables x 2)
            %   z - the lifted state at the current timestep
            
            % make sure shape bounds are included if the system needs them
            if ( nargin < 4 ) && strcmp( obj.shape_bounds , 'on' )
                error('This system has shape bounds so you must include a shape bounds argument');
            elseif nargin < 4
                shape_bounds = [];  % null value
            end
            
            % shorthand variable names
            Np = obj.horizon;       % steps in the horizon
            nd = obj.params.nd;     % number of delays
            
            % construct the current value of zeta
            [ ~ , zeta_temp ] = obj.get_zeta( traj );
            zeta = zeta_temp( end , : )';   % want most recent points
            
            % lift zeta
            z = obj.lift.full( zeta );
            
            % check that reference trajectory has correct dimensions
            if size( ref , 2 ) ~= size( obj.projmtx , 1 )
                error('Reference trajectory is not the correct dimension');
            elseif size( ref , 1 ) > Np + 1
                ref = ref( 1 : Np + 1 , : );    % remove points over horizon
            elseif size( ref , 1 ) < Np + 1
                ref_temp = kron( ones( Np+1 , 1 ) , ref(end,:) );
                ref_temp( 1 : size(ref,1) , : ) = ref;
                ref = ref_temp;     % repeat last point for remainer of horizon
            end
            
            % vectorize the reference trajectory
            Yr = reshape( ref' , [ ( Np + 1 ) * size(ref,2) , 1 ] );
            
            % get the RHS of the inequality constraints
            c = obj.constraints.c;
            
            % setup matrices for gurobi solver
            H = obj.cost.H;      % removed factor of 2 on 12/10/2018
            f = ( z' * obj.cost.G + Yr' * obj.cost.D )';
            A = obj.constraints.L;
            b = - obj.constraints.M * z + c;
            
%             % tack on "memory" constraint to fix initial input u_0
%             Atack = [ [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - obj.params.m ) ];
% %             Atack_bot = [ sparse( 2*obj.params.m , obj.params.m) , [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - 2*obj.params.m ) ];
% %             Atack = [ Atack_top ; Atack_bot ];
%             if isfield( traj , 'unl' )
%                 btack = [ traj.unl(end,:)' ; -traj.unl(end,:)' ];
%             else
%                 btack = [ traj.u(end,:)' ; -traj.u(end,:)' ];
%             end
%             A = [A ; Atack];    % tack on memory constraint
%             b = [b ; btack];
            
            % solve the MPC problem
            Uvec = quadprog_gurobi( 2*H , f , A , b );   % solve using gurobi (returns NaNs of cannot be solved)
            % U = quadprog( 2*H , f , A , b );     % solve using matlab
            
            % reshape the output so each input will have one row (first row equals current input)
            U = reshape( Uvec , [ obj.params.m , Np ] )';
        end
        
        %% nonlinear input (probably will delete this section)
        
        % get_mpcInput_unl: Solve the mpc problem to get the input over entire horizon
        function [ E , Z ]= get_mpcInput_unl( obj , traj , ref , shape_bounds )
            %get_mpcInput_unl: Soves the mpc problem to get the input over
            % entire horizon for a system with a nonlinear input
            %   traj - struct with fields y , u. Contains the measured
            %    states and inputs for the past ndelay+1 timesteps.
            %   ref - matrix containing the reference trajectory for the
            %    system over the horizon (one row per timestep).
            %   shape_bounds - [min_shape_parameters , max_shape_parameters] 
            %    This is only requred if system has shape constraints 
            %   (note: size is num of shape observables x 2)
            %   z - the lifted state at the current timestep
            
            % make sure shape bounds are included if the system needs them
            if ( nargin < 4 ) && strcmp( obj.shape_bounds , 'on' )
                error('This system has shape bounds so you must include a shape bounds argument');
            elseif nargin < 4
                shape_bounds = [];  % null value
            end
            
            % shorthand variable names
            Np = obj.horizon;       % steps in the horizon
            nd = obj.params.nd;     % number of delays
            
            % isolate most recent output
            statenow = traj( end , : );
            
            % lift zeta
            zx = obj.lift.zx( statenow.y' );
            
            % check that reference trajectory has correct dimensions
            if size( ref , 2 ) ~= obj.params.n    % size( obj.projmtx , 1 )
                error('Reference trajectory is not the correct dimension');   % removed just for now ( 2019-10-19 )
            elseif size( ref , 1 ) > Np + 1
                ref = ref( 1 : Np + 1 , : );    % remove points over horizon
            elseif size( ref , 1 ) < Np + 1
                ref_temp = kron( ones( Np+1 , 1 ) , ref(end,:) );
                ref_temp( 1 : size(ref,1) , : ) = ref;
                ref = ref_temp;     % repeat last point for remainer of horizon
            end
            
            % lift the reference trajectory (try to vectorize by modifying the zx lifting function
            ref_zx = zeros( size(ref,1) , length(zx) );
            for i = 1 : size( ref , 1 )
                ref_zx(i,:) = obj.lift.zx( ref(i,:)' )';
            end
            
            % vectorize the reference trajectory
            Yr = reshape( ref_zx' , [ ( Np + 1 ) * size(ref_zx,2) , 1 ] );
            
            % get the RHS of the inequality constraints
            c = obj.set_constRHS( shape_bounds );
            
            % setup matrices for gurobi solver
            H = obj.cost.H;      % removed factor of 2 on 12/10/2018
            f = ( zx' * obj.cost.G + Yr' * obj.cost.D )';
            A = obj.constraints.L;
            b = - obj.constraints.M * zx + c;
            
            % tack on "memory" constraint to fix initial input u_0
            Atack = [ [ speye( obj.params.nzx ) ; -speye( obj.params.nzx ) ] , sparse( 2*obj.params.nzx , size(A,2) - obj.params.nzx ) ];
%             Atack_bot = [ sparse( 2*obj.params.m , obj.params.m) , [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - 2*obj.params.m ) ];
%             Atack = [ Atack_top ; Atack_bot ];
            if isfield( traj , 'e' )
                btack = [ traj.e(end,:)' ; -traj.e(end,:)' ];
            else
                btack = [ traj.u(end,:)' ; -traj.u(end,:)' ];
            end
            A = [A ; Atack];    % tack on memory constraint
            b = [b ; btack];
            
            % solve the MPC problem
            Evec = quadprog_gurobi( 2*H , f , A , b );   % solve using gurobi (returns NaNs of cannot be solved)
            % U = quadprog( 2*H , f , A , b );     % solve using matlab
            
            % reshape the output so each input will have one row (first row equals current input)
            E = reshape( Evec , [ obj.params.nzx , Np ] )';
            
            % calculate the projected lifted state over the horizon with the given inputs
            Zvec = obj.cost.A * zx + obj.cost.B * Evec;
            Z = reshape( Zvec( obj.params.nzx + 1 : end ) , [ obj.params.nzx , Np ] )';
        end
        
        % get_costMatrices_unl: Contructs the matrices for the mpc optim. problem
        function obj = get_costMatrices_unl( obj )
            %get_costMatrices_unl: Constructs cost the matrices for the mpc 
            % optimization problem.
            %   obj.cost has fields H, G, D, A, B, C, Q, R
            
            % define cost function matrices
            % Cost function is defined: U'HU + ( z0'G + Yr'D )U
            
            model = obj.model;
            
            % A
            N = size(model.A,1);
            A = sparse( N*(obj.horizon+1) , N );
            for i = 0 : obj.horizon
                A( (N*i + 1) : N*(i+1) , : ) = model.A^i ;
            end
            
            % B
            Bhere = speye( N ); % identity matrix so MPC will output "error" as the pseudoinput
            Bheight = N*(obj.horizon+1);
            Bcolwidth = size(Bhere,2);
            Bcol = sparse( Bheight , Bcolwidth );    % first column of B matrix
            for i = 1 : obj.horizon
                Bcol( (N*i + 1) : N*(i+1) , : ) = model.A^(i-1) * Bhere ;
            end
            
            Lshift = spdiags( ones( N*obj.horizon , 1 ) , -N , N*(obj.horizon+1) , N*(obj.horizon+1) );    % lower shift operator
            
            Bwidth = size(Bhere,2)*(obj.horizon);    % total columns in B matrix
            Bblkwidth = obj.horizon;   % how many Bcol blocks wide B matrix is
            B = spalloc( Bheight , Bwidth , floor(Bheight * Bwidth / 2) ); % initialze sparse B matrix
            B(: , 1:Bcolwidth) = Bcol;
            for i = 2 : Bblkwidth
                B(: , (i-1)*Bcolwidth+1 : i*Bcolwidth) = Lshift * B(: , (i-2)*Bcolwidth+1 : (i-1)*Bcolwidth);
            end
            
            % C: matrix that projects lifted state into reference trajectory space
            C = kron( speye(obj.horizon+1) , obj.projmtx);
            nproj = size( obj.projmtx , 1 );
            
            % Q: Error magnitude penalty
            Q = kron( speye(obj.horizon+1) , eye(nproj) * obj.cost_running); % error magnitude penalty (running cost) (default 0.1)
            Q(end-nproj+1 : end , end-nproj+1 : end) = eye(nproj) * obj.cost_terminal;    % (terminal cost) (default 100)
            
            % R: Input magnitude penalty
            R = kron( speye(obj.horizon) , eye(model.params.m) * obj.cost_input );  % input magnitude penalty (for flaccy use 0.5e-2) (new videos used 0.5e-3)

            % H, G, D
            H = B' * C' * Q * C * B + R;
            G = 2 * A' * C' * Q * C * B;
            D = -2 * Q * C * B;
            
            % set outputs
            obj.cost.H = H; obj.cost.G = G; obj.cost.D = D; % constructed matrices
            obj.cost.A = A; obj.cost.B = B; obj.cost.C = C; obj.cost.Q = Q; obj.cost.R = R; % component matrices
        end
        
        % get_constraintMatrices_unl: Constructs the constraint matrices
        function obj = get_constraintMatrices_unl( obj )
            %get_constraintMatrices: Constructs the constraint matrices for
            % the mpc optimization problem.
            %   obj.constraints has fields L, M, F, E, (c?)
            %   F is for input constraints
            %   E is for state constraints
            
            % shorten some variable names
            Np = obj.horizon;     % steps in horizon
            params = obj.params;    % linear system model parameters
            cost = obj.cost;     % cost matrices
            
            F = []; E = [];     % initialize empty matrices
            c = sym( [] );      % RHS to be defined symbolically
            
            % input_bounds
            if ~isempty( obj.input_bounds )
                num = 2*params.m;     % number of input bound constraints
                
                % F: input_bounds
                Fbounds_i = [ -speye(params.m) ; speye(params.m) ];    % diagonal element of F, for bounded inputs
                Fbounds = sparse( num * (Np+1) , size(cost.B,2) );  % no constraints, all zeros
                Fbounds( 1:num*Np , 1:Np*params.m ) = kron( speye(Np) , Fbounds_i );     % fill in nonzeros
                F = [ F ; Fbounds ];    % append matrix
                
                % E: input_bounds (just zeros)
                Ebounds = sparse( num * (Np+1) , size(cost.B,1) );  % no constraints, all zeros
                E = [ E ; Ebounds ];    % append matrix
                
                % c: input_bounds
                if isfield( obj.params , 'NLinput' )    % don't scale input if it's nonlinear
                    input_bounds_sc = obj.input_bounds;
                else
                    input_bounds_sc = obj.scaledown.u( obj.input_bounds' )';   % scaled down the input bounds
                end
                cbounds_i = sym( [ -input_bounds_sc(:,1) ; input_bounds_sc(:,2) ] ); % [ -umin ; umax ]
                cbounds = sym( zeros( num * (Np+1) , 1) );    % initialization
                cbounds(1 : num*Np) = kron( ones( Np , 1 ) , cbounds_i );     % fill in nonzeros
                c = [ c ; cbounds ];    % append vector
            end
            
            % input_slopeConst
            if ~isempty( obj.input_slopeConst )
                % F: input_slopeConst
                Fslope_i = speye(params.m);
                Fslope_neg = [ kron( speye(Np-1) , -Fslope_i ) , sparse( params.m * (Np-1) , params.m ) ];
                Fslope_pos = [ sparse( params.m * (Np-1) , params.m ) , kron( speye(Np-1) , Fslope_i ) ];
                Fslope_top = Fslope_neg + Fslope_pos;
                Fslope = [ Fslope_top ; -Fslope_top];
                F = [ F ; Fslope ];     % append matrix

                % E: input_slopeConst (just zeros)
                E = [ E ; sparse( 2 * params.m * (Np-1) , size(cost.B,1) ) ];
                
                % c: input_slopeConst
                if isfield( obj.params , 'NLinput' )    % don't scale slope if it's nonlinear
                    slope_lim = obj.input_slopeConst;
                else
                    slope_lim = obj.input_slopeConst * mean( params.scale.u_factor );  % scale down the 2nd deriv. limit
                end
                cslope_top = sym( slope_lim * ones( params.m * (Np-1) , 1 ) );
                cslope = [ cslope_top ; cslope_top ];
                c = [ c ; cslope ];     % append vector
            end
            
            % input_smoothConst
            if ~isempty( obj.input_smoothConst )
                % F: input_smoothConst
                Fsmooth_i = speye(params.m);
                Fsmooth_lI = [ kron( speye(Np-2) , Fsmooth_i ) , sparse( params.m * (Np-2) , 2 * params.m ) ];
                Fsmooth_2I = [ sparse( params.m * (Np-2) , params.m ) , kron( speye(Np-2) , -2*Fslope_i ) , sparse( params.m * (Np-2) , params.m ) ];
                Fsmooth_rI = [ sparse( params.m * (Np-2) , 2 * params.m ) , kron( speye(Np-2) , Fslope_i ) ];
                Fsmooth_top = Fsmooth_lI + Fsmooth_2I + Fsmooth_rI;
                Fsmooth = [ Fsmooth_top ; -Fsmooth_top ];
                F = [ F ; Fsmooth ];
                
                % E: input_smoothConst
                E = [ E ; sparse( 2 * params.m * (Np-2) , size(cost.B,1) ) ];
                
                % c: input_smoothConst
                smooth_lim = params.Ts^2 * obj.input_smoothConst * mean( params.scale.u_factor );  % scale down the 2nd deriv. limit
                csmooth = sym( smooth_lim * ones( size(Fsmooth,1) ,1) );
                c = [ c ; csmooth ];
            end
            
            % state_bounds
            if ~isempty( obj.state_bounds )
                num = 2*params.n;   % number of state bound constraints
                
                % E: state_bounds
                Esbounds_i = [ -speye(params.n) ; speye(params.n) ];    % diagonal element of E, for bounding low dim. states (first n elements of lifted state)
                Esbounds = sparse( num * (Np+1) , size(cost.A,1) );  % no constraints, all zeros
                Esbounds( 1:num*(Np+1) , 1:(Np+1)*params.n ) = kron( speye(Np+1) , Esbounds_i );     % fill in nonzeros
                E = [ E ; Esbounds ];    % append matrix
                
                % F: state_bounds (all zeros)
                Fsbounds = zeros( size( Esbounds , 1 ) , size( cost.B , 2 ) );
                F = [ F ; Fsbounds ];    % append matrix
                
                % c: state_bounds
                state_bounds_sc = obj.scaledown.y( obj.state_bounds' )';    % scaled down state bounds
                csbounds_i = sym( [ -state_bounds_sc(:,1) ; state_bounds_sc(:,2) ] ); % [ -ymin ; ymax ]
                csbounds = kron( ones( Np+1 , 1 ) , csbounds_i );     % fill in nonzeros
                c = [ c ; csbounds ];    % append vector
            end
            
            % shape_bounds (imposes bounds on shape parameters over entire horizon)
            if strcmp( obj.shape_bounds , 'on' )
                
                % E: shape_bounds
                Eshape_i = [ -obj.model.Cshape ; obj.model.Cshape ];    % isolates the shape observables
                Eshape = kron( speye(Np+1) , Eshape_i );    % combine
                E = [ E ; Eshape ];     % append matrix
                
                % F: shape_bounds (all zeros)
                Fshape = zeros( size( Eshape , 1 ) , size( cost.B , 2 ) );
                F = [ F ; Fshape ];    % append matrix
                
                % VERSION 1: enforces same constraint over entire horizon (incompatible with current form of the shape_bounds input)
                % c: shape_bounds (symbolic variables)
%                 coeffs_bounds = sym( 'coeffs_bounds' , [ size( obj.model.Cshape , 1 ) , 2 ] , 'real');   % [ min , max ]
%                 cshape_i = [ -coeffs_bounds(:,1) ; coeffs_bounds(:,2) ];
%                 cshape = kron( ones( Np+1 , 1 ) , cshape_i );
                % VERSION 2: revamped for shape constraints that evolve over time
                coeffs_bounds = sym( 'coeffs_bounds' , [ Np + 1 , 2 * size( obj.model.Cshape , 1 ) ] , 'real');   % [ min , max ]
                len_cb = size(coeffs_bounds,2);
                cshape = inf( size( coeffs_bounds , 2 ) , 1 ); % don't worry about constraint violation at current timestep
                for i = 1 : Np
                    cshape_i = [ -coeffs_bounds( i , 1 : len_cb/2 ) , coeffs_bounds( i , len_cb/2 + 1 : len_cb ) ]';
                    cshape = [ cshape ; cshape_i ];
                end
                % VERSION 3: only enforce as a terminal constraint (could do this without infs if I also change E matrix)
%                 coeffs_bounds = sym( 'coeffs_bounds' , [ Np + 1 , 2 * size( obj.model.Cshape , 1 ) ] , 'real');   % [ min , max ]
%                 len_cb = size(coeffs_bounds,2);
%                 cshape = inf( Np * size( coeffs_bounds , 2 ) , 1 ); % don't worry about constraint violation at current timestep
%                 cshape_end = [ -coeffs_bounds( Np + 1 , 1 : len_cb/2 ) , coeffs_bounds( Np + 1 , len_cb/2 + 1 : len_cb ) ]';
%                 cshape = [ cshape ; cshape_end ];
                
                c = [ c ; cshape ];
                
                % define set_constRHS, which sets the value of c
                obj.set_constRHS = matlabFunction( c , 'Vars', { coeffs_bounds } );
                
            else    % if there is no shape constraints, set_constRHS function should have dummy input
                empty_sym = sym([]);
                
                % define set_constRHS, which sets the value of c
                obj.set_constRHS = matlabFunction( c , 'Vars', { empty_sym } );
            end
            
            % set outputs
            obj.constraints.F = F;
            obj.constraints.E = E;    
            obj.constraints.c = c;
            obj.constraints.L = F + E * cost.B;
            obj.constraints.M = E * cost.A;
        end
        
        %% bilinear mpc functions
        
         % get_costMatrices_bilinear: Contructs the matrices for the mpc optim. problem
        function obj = get_costMatrices_bilinear( obj )
            %get_costMatrices_unl: Constructs cost the matrices for the mpc 
            % optimization problem.
            %   obj.cost has fields H, G, D, A, B, C, Q, R
            
            % define cost function matrices
            % Cost function is defined: U'HU + ( z0'G + Yr'D )U
            
            model = obj.model;
            
            % A
            N = size(model.A,1);
            A = sparse( N*(obj.horizon+1) , N );
            for i = 0 : obj.horizon
                A( (N*i + 1) : N*(i+1) , : ) = model.A^i ;
            end
            
            % B (model) symbolic and function
            zxblock = kron( eye( obj.params.m ) , obj.basis.zx );   % symbolic block matrix of zx
            Bmodel_sym = obj.model.B * zxblock;
            obj.model.get_B = matlabFunction( Bmodel_sym , 'Vars' , {obj.params.x} ); % function for evaluating value of B matrix at a given state
            
            % B (mpc) zeros, just for size
            Bheight = N*(obj.horizon+1);
%             Bcolwidth = obj.params.m;
%             Bcol = sym( zeros(Bheight , Bcolwidth) );   % first column of B matrix
            obj.cost.randB = rand( size( model.B * zxblock ) );   % DEBUG: Random B matrix
%             for i = 1 : obj.horizon
%                 Bcol( (N*i + 1) : N*(i+1) , : ) = model.A^(i-1) * model.B * zxblock ;   % this is a symbolic expression
% %                 Bcol( (N*i + 1) : N*(i+1) , : ) = model.A^(i-1) * obj.cost.randB ;   % DEBUG: Random B matrix
%             end
%             
%             Lshift = spdiags( ones( N*obj.horizon , 1 ) , -N , N*(obj.horizon+1) , N*(obj.horizon+1) );    % lower shift operator
%             
            Bwidth = obj.params.m * (obj.horizon);    % total columns in B matrix
%             Bblkwidth = obj.horizon;   % how many Bcol blocks wide B matrix is
            B = zeros( Bheight , Bwidth ); % initialze sparse B matrix
%             B(: , 1:Bcolwidth) = Bcol;
%             for i = 2 : Bblkwidth
%                 B(: , (i-1)*Bcolwidth+1 : i*Bcolwidth) = Lshift * B(: , (i-2)*Bcolwidth+1 : (i-1)*Bcolwidth);
%             end
            
            % C: matrix that projects lifted state into reference trajectory space
            C = kron( speye(obj.horizon+1) , obj.projmtx);
            nproj = size( obj.projmtx , 1 );
            
            % Q: Error magnitude penalty
            Q = kron( speye(obj.horizon+1) , eye(nproj) * obj.cost_running); % error magnitude penalty (running cost) (default 0.1)
            Q(end-nproj+1 : end , end-nproj+1 : end) = eye(nproj) * obj.cost_terminal;    % (terminal cost) (default 100)
            
            % R: Input magnitude penalty
            R = kron( speye(obj.horizon) , eye(model.params.m) * obj.cost_input );  % input magnitude penalty (for flaccy use 0.5e-2) (new videos used 0.5e-3)

%             % H, G, D
%             H = B' * C' * Q * C * B + R;
%             G = 2 * A' * C' * Q * C * B;
%             D = -2 * Q * C * B;
            
            % set outputs
%             obj.cost.H = H; obj.cost.G = G; obj.cost.D = D; obj.cost.B = B; % constructed matrices
            obj.cost.A = A; obj.cost.C = C; obj.cost.Q = Q; obj.cost.R = R; % component matrices
            obj.cost.B = B; % all zeros, just included for size
            
            obj.cost.get_B = @obj.get_costB_bilinear;%  matlabFunction( B , 'Vars' , {obj.params.x} ); % function for evaluating value of B matrix at a given state
            obj.cost.get_H = @obj.get_costH_bilinear;%matlabFunction( H , 'Vars' , {obj.params.x} ); % function for evaluating value of H matrix at a given state
            obj.cost.get_G = @obj.get_costG_bilinear;%matlabFunction( G , 'Vars' , {obj.params.x} ); % function for evaluating value of G matrix at a given state
            obj.cost.get_D = @obj.get_costD_bilinear;%matlabFunction( D , 'Vars' , {obj.params.x} ); % function for evaluating value of D matrix at a given state
        end
        
        % get_modelB_bilinear: Constructs the model state dependent B matrix
        function B = get_modelB_bilinear( obj , x )
            zx = obj.lift.zx( x );
            zxblock = kron( eye( obj.params.m ) , zx );   % block matrix of zx
            B = obj.model.B * zxblock;
        end
        
        % get_costB_bilinear: Constructs the mpc state dependent B matrix
        function B = get_costB_bilinear( obj , x )
            % x is row vector with 1 row or same number of rows as horizon
            N = size(obj.model.A,1);
            % B
%             zx = obj.lift.zx( x );
%             zxblock = kron( eye( obj.params.m ) , zx );   % block matrix of zx  
            Bheight = N*(obj.horizon+1);
            Bcolwidth = obj.params.m;
            Bcol = zeros(Bheight , Bcolwidth);   % first column of B matrix
%             obj.cost.randB = rand( size( model.B * zxblock ) );   % DEBUG: Random B matrix
            for i = 1 : obj.horizon
                if size(x,1) > 1
                    Bmodel = obj.model.get_B( x(i,:)' );
                else
                    Bmodel = obj.model.get_B( x(1,:)' );
                end
                Bcol( (N*i + 1) : N*(i+1) , : ) = obj.model.A^(i-1) * Bmodel ;   % this is a symbolic expression
%                 Bcol( (N*i + 1) : N*(i+1) , : ) = obj.model.A^(i-1) * obj.model.B * zxblock ;   % this is a symbolic expression
%                 Bcol( (N*i + 1) : N*(i+1) , : ) = obj.model.A^(i-1) * obj.cost.randB ;   % DEBUG: Random B matrix
            end
            
            Lshift = spdiags( ones( N*obj.horizon , 1 ) , -N , N*(obj.horizon+1) , N*(obj.horizon+1) );    % lower shift operator
            
            Bwidth = obj.params.m * (obj.horizon);    % total columns in B matrix
            Bblkwidth = obj.horizon;   % how many Bcol blocks wide B matrix is
            B = zeros( Bheight , Bwidth ); % initialze sparse B matrix
            B(: , 1:Bcolwidth) = Bcol;
            for i = 2 : Bblkwidth
                B(: , (i-1)*Bcolwidth+1 : i*Bcolwidth) = Lshift * B(: , (i-2)*Bcolwidth+1 : (i-1)*Bcolwidth);
            end
        end
        
        % get_costH_bilinear: Constructs the mpc state dependent H matrix
        function H = get_costH_bilinear( obj , x )
            B = obj.get_costB_bilinear( x );
            C = obj.cost.C;
            Q = obj.cost.Q;
            R = obj.cost.R;
            H = B' * C' * Q * C * B + R;
        end
        
        % get_costG_bilinear: Constructs the mpc state dependent G matrix
        function G = get_costG_bilinear( obj , x )
            B = obj.get_costB_bilinear( x );
            C = obj.cost.C;
            Q = obj.cost.Q;
            A = obj.cost.A;
            G = 2 * A' * C' * Q * C * B;
        end
        
        % get_costD_bilinear: Constructs the mpc state dependent D matrix
        function D = get_costD_bilinear( obj , x )
            B = obj.get_costB_bilinear( x );
            C = obj.cost.C;
            Q = obj.cost.Q;
            D = -2 * Q * C * B;
        end
        
        % get_constraintMatrices_bilinear: Constructs the constraint matrices
        function obj = get_constraintMatrices_bilinear( obj )
            %get_constraintMatrices: Constructs the constraint matrices for
            % the mpc optimization problem.
            %   obj.constraints has fields L, M, F, E, (c?)
            %   F is for input constraints
            %   E is for state constraints
            
            % shorten some variable names
            Np = obj.horizon;     % steps in horizon
            params = obj.params;    % linear system model parameters
            cost = obj.cost;     % cost matrices
            
            F = []; E = [];     % initialize empty matrices
            c = sym( [] );      % RHS to be defined symbolically
            
            % input_bounds
            if ~isempty( obj.input_bounds )
                num = 2*params.m;     % number of input bound constraints
                
                % F: input_bounds
                Fbounds_i = [ -speye(params.m) ; speye(params.m) ];    % diagonal element of F, for bounded inputs
                Fbounds = sparse( num * (Np+1) , size(cost.B,2) );  % no constraints, all zeros
                Fbounds( 1:num*Np , 1:Np*params.m ) = kron( speye(Np) , Fbounds_i );     % fill in nonzeros
                F = [ F ; Fbounds ];    % append matrix
                
                % E: input_bounds (just zeros)
                Ebounds = sparse( num * (Np+1) , size(cost.B,1) );  % no constraints, all zeros
                E = [ E ; Ebounds ];    % append matrix
                
                % c: input_bounds
                if isfield( obj.params , 'NLinput' )    % don't scale input if it's nonlinear
                    input_bounds_sc = obj.input_bounds;
                else
                    input_bounds_sc = obj.scaledown.u( obj.input_bounds' )';   % scaled down the input bounds
                end
                cbounds_i = sym( [ -input_bounds_sc(:,1) ; input_bounds_sc(:,2) ] ); % [ -umin ; umax ]
                cbounds = sym( zeros( num * (Np+1) , 1) );    % initialization
                cbounds(1 : num*Np) = kron( ones( Np , 1 ) , cbounds_i );     % fill in nonzeros
                c = [ c ; cbounds ];    % append vector
            end
            
            % input_slopeConst
            if ~isempty( obj.input_slopeConst )
                % F: input_slopeConst
                Fslope_i = speye(params.m);
                Fslope_neg = [ kron( speye(Np-1) , -Fslope_i ) , sparse( params.m * (Np-1) , params.m ) ];
                Fslope_pos = [ sparse( params.m * (Np-1) , params.m ) , kron( speye(Np-1) , Fslope_i ) ];
                Fslope_top = Fslope_neg + Fslope_pos;
                Fslope = [ Fslope_top ; -Fslope_top];
                F = [ F ; Fslope ];     % append matrix

                % E: input_slopeConst (just zeros)
                E = [ E ; sparse( 2 * params.m * (Np-1) , size(cost.B,1) ) ];
                
                % c: input_slopeConst
                if isfield( obj.params , 'NLinput' )    % don't scale slope if it's nonlinear
                    slope_lim = obj.input_slopeConst;
                else
                    slope_lim = obj.input_slopeConst * mean( params.scale.u_factor );  % scale down the 2nd deriv. limit
                end
                cslope_top = sym( slope_lim * ones( params.m * (Np-1) , 1 ) );
                cslope = [ cslope_top ; cslope_top ];
                c = [ c ; cslope ];     % append vector
            end
            
            % input_smoothConst
            if ~isempty( obj.input_smoothConst )
                % F: input_smoothConst
                Fsmooth_i = speye(params.m);
                Fsmooth_lI = [ kron( speye(Np-2) , Fsmooth_i ) , sparse( params.m * (Np-2) , 2 * params.m ) ];
                Fsmooth_2I = [ sparse( params.m * (Np-2) , params.m ) , kron( speye(Np-2) , -2*Fslope_i ) , sparse( params.m * (Np-2) , params.m ) ];
                Fsmooth_rI = [ sparse( params.m * (Np-2) , 2 * params.m ) , kron( speye(Np-2) , Fslope_i ) ];
                Fsmooth_top = Fsmooth_lI + Fsmooth_2I + Fsmooth_rI;
                Fsmooth = [ Fsmooth_top ; -Fsmooth_top ];
                F = [ F ; Fsmooth ];
                
                % E: input_smoothConst
                E = [ E ; sparse( 2 * params.m * (Np-2) , size(cost.B,1) ) ];
                
                % c: input_smoothConst
                smooth_lim = params.Ts^2 * obj.input_smoothConst * mean( params.scale.u_factor );  % scale down the 2nd deriv. limit
                csmooth = sym( smooth_lim * ones( size(Fsmooth,1) ,1) );
                c = [ c ; csmooth ];
            end
            
            % state_bounds
            if ~isempty( obj.state_bounds )
                num = 2*params.n;   % number of state bound constraints
                
                % E: state_bounds
                Esbounds_i = [ -speye(params.n) ; speye(params.n) ];    % diagonal element of E, for bounding low dim. states (first n elements of lifted state)
                Esbounds = sparse( num * (Np+1) , size(cost.A,1) );  % no constraints, all zeros
                Esbounds( 1:num*(Np+1) , 1:(Np+1)*params.n ) = kron( speye(Np+1) , Esbounds_i );     % fill in nonzeros
                E = [ E ; Esbounds ];    % append matrix
                
                % F: state_bounds (all zeros)
                Fsbounds = zeros( size( Esbounds , 1 ) , size( cost.B , 2 ) );
                F = [ F ; Fsbounds ];    % append matrix
                
                % c: state_bounds
                state_bounds_sc = obj.scaledown.y( obj.state_bounds' )';    % scaled down state bounds
                csbounds_i = sym( [ -state_bounds_sc(:,1) ; state_bounds_sc(:,2) ] ); % [ -ymin ; ymax ]
                csbounds = kron( ones( Np+1 , 1 ) , csbounds_i );     % fill in nonzeros
                c = [ c ; csbounds ];    % append vector
            end
            
            % shape_bounds (imposes bounds on shape parameters over entire horizon)
            if strcmp( obj.shape_bounds , 'on' )
                
                % E: shape_bounds
                Eshape_i = [ -obj.model.Cshape ; obj.model.Cshape ];    % isolates the shape observables
                Eshape = kron( speye(Np+1) , Eshape_i );    % combine
                E = [ E ; Eshape ];     % append matrix
                
                % F: shape_bounds (all zeros)
                Fshape = zeros( size( Eshape , 1 ) , size( cost.B , 2 ) );
                F = [ F ; Fshape ];    % append matrix
                
                % VERSION 1: enforces same constraint over entire horizon (incompatible with current form of the shape_bounds input)
                % c: shape_bounds (symbolic variables)
%                 coeffs_bounds = sym( 'coeffs_bounds' , [ size( obj.model.Cshape , 1 ) , 2 ] , 'real');   % [ min , max ]
%                 cshape_i = [ -coeffs_bounds(:,1) ; coeffs_bounds(:,2) ];
%                 cshape = kron( ones( Np+1 , 1 ) , cshape_i );
                % VERSION 2: revamped for shape constraints that evolve over time
                coeffs_bounds = sym( 'coeffs_bounds' , [ Np + 1 , 2 * size( obj.model.Cshape , 1 ) ] , 'real');   % [ min , max ]
                len_cb = size(coeffs_bounds,2);
                cshape = inf( size( coeffs_bounds , 2 ) , 1 ); % don't worry about constraint violation at current timestep
                for i = 1 : Np
                    cshape_i = [ -coeffs_bounds( i , 1 : len_cb/2 ) , coeffs_bounds( i , len_cb/2 + 1 : len_cb ) ]';
                    cshape = [ cshape ; cshape_i ];
                end
                % VERSION 3: only enforce as a terminal constraint (could do this without infs if I also change E matrix)
%                 coeffs_bounds = sym( 'coeffs_bounds' , [ Np + 1 , 2 * size( obj.model.Cshape , 1 ) ] , 'real');   % [ min , max ]
%                 len_cb = size(coeffs_bounds,2);
%                 cshape = inf( Np * size( coeffs_bounds , 2 ) , 1 ); % don't worry about constraint violation at current timestep
%                 cshape_end = [ -coeffs_bounds( Np + 1 , 1 : len_cb/2 ) , coeffs_bounds( Np + 1 , len_cb/2 + 1 : len_cb ) ]';
%                 cshape = [ cshape ; cshape_end ];
                
                c = [ c ; cshape ];
                
                % define set_constRHS, which sets the value of c
                obj.set_constRHS = matlabFunction( c , 'Vars', { coeffs_bounds } );
                
            else    % if there is no shape constraints, set_constRHS function should have dummy input
                empty_sym = sym([]);
                
                % define set_constRHS, which sets the value of c
                obj.set_constRHS = matlabFunction( c , 'Vars', { empty_sym } );
            end
            
            % set outputs
            obj.constraints.F = F;
            obj.constraints.E = E;    
            obj.constraints.c = c;
%             obj.constraints.L = F + E * cost.B;
            obj.constraints.get_L = @obj.get_constraintL_bilinear; %matlabFunction( obj.constraints.L , 'Vars' , {obj.params.x} ); % function for evaluating value of L matrix at a given state
            obj.constraints.M = E * cost.A;
        end
        
        % get_costD_bilinear: Constructs the mpc state dependent D matrix
        function L = get_constraintL_bilinear( obj , x )
            B = obj.get_costB_bilinear( x );
            F = obj.constraints.F;
            E = obj.constraints.E;
            L = F + E * B;
        end
        
        % get_mpcInput_bilinear: Solve the mpc problem to get the input over entire horizon
        function [ E , Z ]= get_mpcInput_bilinear( obj , traj , ref , shape_bounds )
            %get_mpcInput_unl: Soves the mpc problem to get the input over
            % entire horizon for a system with a nonlinear input
            %   traj - struct with fields y , u. Contains the measured
            %    states and inputs for the past ndelay+1 timesteps.
            %   ref - matrix containing the reference trajectory for the
            %    system over the horizon (one row per timestep).
            %   shape_bounds - [min_shape_parameters , max_shape_parameters] 
            %    This is only requred if system has shape constraints 
            %   (note: size is num of shape observables x 2)
            %   z - the lifted state at the current timestep
            
            % make sure shape bounds are included if the system needs them
            if ( nargin < 4 ) && strcmp( obj.shape_bounds , 'on' )
                error('This system has shape bounds so you must include a shape bounds argument');
            elseif nargin < 4
                shape_bounds = [];  % null value
            end
            
            % shorthand variable names
            Np = obj.horizon;       % steps in the horizon
            nd = obj.params.nd;     % number of delays
            
            % isolate most recent output
            statenow = traj.y( end , : );   % row
            inputnow = traj.u( end , : );   % row
            
            % lift zeta
            zx = obj.lift.zx( statenow' );
            
            % simulate to find next zx given input u0
            Bz0 = obj.model.get_B( statenow' );
            zx1 = obj.model.A * zx + Bz0 * inputnow';
            statenext = ( obj.model.C * zx1 )';  % row
            
            % check that reference trajectory has correct dimensions
            if size( ref , 2 ) ~= obj.params.n    % size( obj.projmtx , 1 )
                error('Reference trajectory is not the correct dimension');   % removed just for now ( 2019-10-19 )
            elseif size( ref , 1 ) > Np + 1
                ref = ref( 1 : Np + 1 , : );    % remove points over horizon
            elseif size( ref , 1 ) < Np + 1
                ref_temp = kron( ones( Np+1 , 1 ) , ref(end,:) );
                ref_temp( 1 : size(ref,1) , : ) = ref;
                ref = ref_temp;     % repeat last point for remainer of horizon
            end
            
          
            % lift the reference trajectory (try to vectorize by modifying the zx lifting function
            ref_zx = zeros( size(ref,1) , length(zx) );
            for i = 1 : size( ref , 1 )
                ref_zx(i,:) = obj.lift.zx( ref(i,:)' )';
            end
%             ref_zx = ref;   % DEBUG: THIS ASSUMES A PRE-LIFTED INPUT
            
            % vectorize the reference trajectory
            Yr = reshape( ref_zx' , [ ( Np + 1 ) * size(ref_zx,2) , 1 ] );
            
            % get the RHS of the inequality constraints
            c = obj.set_constRHS( shape_bounds );
            
            % setup matrices for gurobi solver
            state_over_horizon = [ statenow ; statenext ; ref(3:end,:) ]; % state over the entire horizon, in the rows  
            H = obj.cost.get_H( state_over_horizon );      % removed factor of 2 on 12/10/2018
            G = obj.cost.get_G( state_over_horizon );
            D = obj.cost.get_D( state_over_horizon );
            f = ( zx' * G + Yr' * D )';
            A = obj.constraints.get_L( state_over_horizon );
            b = - obj.constraints.M * zx + c;
            
            % tack on "memory" constraint to fix initial input u_0
            Atack = [ [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - obj.params.m ) ];
%             Atack_bot = [ sparse( 2*obj.params.m , obj.params.m) , [ speye( obj.params.m ) ; -speye( obj.params.m ) ] , sparse( 2*obj.params.m , size(A,2) - 2*obj.params.m ) ];
%             Atack = [ Atack_top ; Atack_bot ];
            if isfield( traj , 'e' )
                btack = [ traj.e(end,:)' ; -traj.e(end,:)' ];
            else
                btack = [ traj.u(end,:)' ; -traj.u(end,:)' ];
            end
            A = [A ; Atack];    % tack on memory constraint
            b = [b ; btack];
            
            % solve the MPC problem
            [ Evec , results ] = quadprog_gurobi( 2*H , f , A , b );   % solve using gurobi (returns NaNs of cannot be solved)
            % U = quadprog( 2*H , f , A , b );     % solve using matlab
            
            % reshape the output so each input will have one row (first row equals current input)
            E = reshape( Evec , [ obj.params.m , Np ] )';
            
            % calculate the projected lifted state over the horizon with the given inputs (assuming constant Bz0)
            Bz = obj.cost.get_B( state_over_horizon );
            Zvec = obj.cost.A * zx + Bz * Evec;
            Z = reshape( Zvec( obj.params.nzx + 1 : end ) , [ obj.params.nzx , Np ] )';
        end
        
        %% nonlinear MPC functions
    end
end