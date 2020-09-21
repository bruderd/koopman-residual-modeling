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
         
        %% Simulate the system with mpc controller
        
        % run_trial_mpc: Runs a simulation of system under linear mpc controller
        function results = run_trial_mpc( obj , ref , x0 , u0 , load )
            %run_trial: Runs a simulation of system under mpc controller.
            %   Tries to follow the trajectory in ref and impose the
            %   shape constraints in shape_bounds.
            %   Assume ref and shape_bounds have same sampling frequency as
            %   sytem, and that they are already scaled to be consistent 
            %   with the lifted model.
            %   ref - each row is a desired point at the corresponding timestep
            %   x0 - [1,nx] initial condtion
            %   u0 - [1,nu] initial input
            %   load - [1,nw] or [total steps,nw] load condition for trial
            
            % shorthand
            nd = obj.mpc.params.nd;
            Np = obj.mpc.horizon;
            
            % set initial state value
            if isempty(x0)
                x0 = zeros( nd+1 , obj.sys.params.nx );
            else
                x0 = kron( ones( nd+1 , 1 ) , x0 );
            end
            
            % set initial state value
            if isempty(u0)
                u0 = zeros( nd+1 , obj.sys.params.nu );
            else
                u0 = kron( ones( nd+1 , 1 ) , u0 );
            end
            y0 = obj.sys.get_y( x0 );   % initial output
            
            % set value for loading condition
            if obj.mpc.loaded
                if nargin < 5
                    error('Missing argument: The model expects a load condition but none was provided');
                else
                    if size( load , 2 ) ~= obj.sys.params.nw
                        error(['Load argument should have ' , obj.sys.params.nw , ' columns, not ' , size( load , 2) ]);
                    end
                    if isempty(load)
                        w = zeros( size(ref,1) , obj.sys.params.nw );
                    elseif size( load , 1 ) == 1    % assume constant load
                        w = kron( ones( size(ref,1) , 1 ) , load );
                    elseif size( load , 1 ) ~= size( ref , 1 )
                        error('Load argument must have 1 or the same number of rows as ref argument');
                    else
                        w = load;
                    end
                end
            elseif ~isempty(load)   % model doesn't have load, but want to apply one to system
                if size( load , 1 ) == 1    % assume constant load
                    w = kron( ones( size(ref,1) , 1 ) , load );
                elseif size( load , 1 ) ~= size( ref , 1 )
                    error('Load argument must have 1 or the same number of rows as ref argument');
                else
                    w = load;
                end
            end
%             % set value of initial conditions to zero if none provided
%             if nargin < 3
%                 x0 = zeros( nd+1 , obj.sys.params.nx );
%                 u0 = zeros( nd+1 , obj.sys.params.nu );
%             elseif nargin < 4
%                 x0 = kron( ones( nd+1 , 1 ) , x0 );
%                 u0 = zeros( nd+1 , obj.sys.params.nu );
%             else
%                 x0 = kron( ones( nd+1 , 1 ) , x0 );
%                 u0 = kron( ones( nd+1 , 1 ) , u0 );
%             end
%             y0 = obj.sys.get_y( x0 );
            
            % resample and scale the reference trajectory (TODO: FIX THIS AND THE REFERENCE TRAJECTORY FORMAT)
%             ref_Ts = obj.resample_ref( ref );
%             ref_sc = obj.scaledown.y( ref_Ts );
            ref_sc = obj.scaledown.ref( ref );
            
            % set initial condition
            initial.y = obj.scaledown.y(y0);
            initial.u = obj.scaledown.u(u0);
            [ initial , zeta0 ] = obj.mpc.get_zeta( initial );    % LINE NOT NEEDED
            
            % initialize results struct
            results = struct;
            results.T = [ 0 ];
            results.U = [ u0( end , : ) ];
            results.Y = [ y0( end , : ) ];
            results.K = [ 0 ];
            results.R = [ ref(1,:) ];
            results.X = [ x0( end , : ) ];
            results.Z = []; %[ obj.mpc.lift.econ_full( zeta0' )' ]; % lifted states
            results.comp_time = []; % mpc computation time
            results.err = [];   % tracking error (Euclidean distance)
            if obj.mpc.loaded
                results.W = w;  % actual load condition over entire horizon
                results.What = [ zeros( 1 , size(w,2) ) ];    % estimated load condition
            elseif ~isempty(load)
                results.W = w;  % actual load condition over entire horizon
            end
            
            k = 1;
            while k < size( ref_sc , 1 )
                
                % current time
                t = k * obj.mpc.params.Ts;
                
                % get current state and input with delays
                if k == 1
                    current.y = obj.scaledown.y( y0 );   
                    current.u = obj.scaledown.u( u0 ); 
                elseif k < nd + 1
                    y = [ y0( k : end-1 , : ) ; results.Y ];
                    u = [ u0( k : end-1 , : ) ; results.U ];
                    current.y = obj.scaledown.y( y );
                    current.u = obj.scaledown.u( u ); 
                else
                    y = results.Y( end - nd : end , : );
                    u = results.U( end - nd : end , : );
                    current.y = obj.scaledown.y( y ); 
                    current.u = obj.scaledown.u( u ); 
                end
                
                % get current estimate of the load condtion (if applicable)
                if obj.mpc.loaded
                    if k < nd + 2
                        ypast = kron( ones(nd+2,1) , obj.scaledown.y( y0 ) );  % minimum size is nd+2
                        upast = kron( ones(nd+2,1) , obj.scaledown.u( u0 ) );  % minimum size is nd+2
                    elseif k < obj.mpc.load_obs_horizon + 1
                        yp = [ y0( k : end-1 , : ) ; results.Y ];
                        up = [ u0( k : end-1 , : ) ; results.U ];
                        ypast = obj.scaledown.y( yp );
                        upast = obj.scaledown.u( up );
                    else
                        yp = results.Y( end - obj.mpc.load_obs_horizon : end , : );
                        up = results.U( end - obj.mpc.load_obs_horizon : end , : );
                        ypast = obj.scaledown.y( yp );
                        upast = obj.scaledown.u( up );
                    end
                    % choose appropriate estimation function for model type
                    if mod( k , obj.mpc.load_obs_period ) == 0
                        if strcmp( obj.mpc.model_type , 'linear' )
                            current.what = obj.mpc.estimate_load_linear( ypast , upast)';   % NOTE!!!!: Only works no delays for now
                        elseif strcmp( obj.mpc.model_type , 'bilinear' )
                            current.what = obj.mpc.estimate_load_bilinear( ypast , upast)';
                        end
                    else
                        current.what = obj.scaledown.w( results.What(end,:) );
                    end
                    results.What = [ results.What ; obj.scaleup.w( current.what ) ]; % update results struct
                end
                
                % isolate the reference trajectory over the horizon
                if k + Np <= size( ref_sc , 1 )
                    refhor = ref_sc( k : k + Np , :);
                else
                    refhor = ref_sc( k : end , : );     % repeat last entry
                end 
                
                % get optimal input over horizon
                tic
                if strcmp( obj.mpc.model_type , 'linear' )
                    [ U , z ] = obj.mpc.get_mpcInput( current , refhor );
                elseif strcmp( obj.mpc.model_type , 'bilinear' )
%                     [ U , z ] = obj.mpc.get_mpcInput_bilinear( current , refhor );
                    [ U , z ] = obj.mpc.get_mpcInput_bilinear_iter( current , refhor , 1 );
                elseif strcmp( obj.mpc.model_type , 'nonlinear' )
                    [ U , z ] = obj.mpc.get_mpcInput_nonlinear( current , refhor );
                end
                comp_time = toc; % mpc computation time
                
                % if a solution was not found, break out of while loop
                if any( isnan(U) )
                    break;
                end
                
                % isolate input for this step (may need to make it U(1,:)
                u_kp1_sc = U( 2 , : );
                
                % scaleup the input for the results
                u_kp1 = obj.scaleup.u( u_kp1_sc )';
                
%                 % simulate the system over one time-step (Using sysid model)
%                 z_k = z;
%                 u_k_sc = obj.scaledown.u( results.U(end,:) );  % need to use previously calculated input NEED TO MAKE THIS CLEANER!!!
%                 z_kp1 = obj.mpc.model.A * z_k + obj.mpc.model.B * u_k_sc';
%                 x_kp1 = obj.mpc.model.C * z_kp1;  % DON'T CARE ABOUT THIS JUST WANT IT TO WORK
%                 y_kp1_sc = x_kp1;  % output just scaled version of state since model was learned from observations
%                 y_kp1 = obj.scaleup.y( y_kp1_sc' )';  % scale output back up 
                
                % simulate the system over one time-step (using actual system model)
                x_k = results.X( end , : )';
                u_k = results.U(end,:)'; %current.u';
                if obj.mpc.loaded || ~isempty(load)
                    w_k = results.W(k,:)'; % actual load at kth timestep
                    x_kp1 = obj.sys.simulate_Ts( x_k , u_k , w_k );
                else
                    x_kp1 = obj.sys.simulate_Ts( x_k , u_k , [] );
                end
                y_kp1 = obj.sys.get_y( x_kp1' )';
                
                % record updated results
                results.T = [ results.T ; t ];
                results.U = [ results.U ; u_kp1' ];
                results.Y = [ results.Y ; y_kp1' ];
                results.K = [ results.K ; k ];
                results.R = [ results.R ; obj.scaleup.ref( ref_sc( k , : ) ) ];   % note that this is not scaled down
                results.X = [ results.X ; x_kp1' ];
                results.Z = [ results.Z ; z'  ]; % current lifted state
                results.comp_time = [ results.comp_time ; comp_time ];   % mpc computation time
                results.err = [ results.err ; sqrt( sum( ( results.R(end,:) - results.Y(end,:)*obj.mpc.projmtx(:,1:obj.mpc.params.n)' ).^2 ) ) ];
                
                k = k + 1;  % increment step counter
            end
        end
 

    end
end