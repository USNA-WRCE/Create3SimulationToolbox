classdef Create3sim < matlab.mixin.SetGet
    % CREATE3SIM mimics the CREATE3 class providing matched commands
    % and feedback in a simulated environment.
    %
    %   Initialization
    %       sim = Create3sim;
    %
    %   L. DeVries & M. Kutzer, 06Oct2024, USNA

    % Controlled visible properties
    % -> Read only
    properties(GetAccess='public', SetAccess='private')
        fig; %simulation figure handle
        axs; %simulation axes handle
    end
    % -> Read & Write
    properties(GetAccess='public', SetAccess='public')
        checkCollosions (1,1) logical = false; %check collisions between robot and environment
    end

    % Controlled hidden properties
    properties(GetAccess='public', SetAccess='private', Hidden=true)
        node; %TBD
        tmr_updateSim; %simulation update timer object
        tmr_updateVis; %simulation plot update timer object
        period_updateSim = 0.02; %simulation sample period in seconds (default to 50 Hz)
        period_updateVis = 0.05; %simulation plot sample period in seconds (default to 20 Hz)
        t_last_input; %time (s) for last input (used for timeout)
        t_last_call = repmat(datetime('now'),1,2); %1x2 datetime array specifying information about last setVelCmd call
        t_now; %current simulation time (s)
        t_timeout = 1; %timeout duration for commands (s)
        maxLinearSpeed = 0.306; %maximum default Create3 linear speed (m/s)
        maxAngularSpeed = 2.64; %maximum default Create3 angular speed (rad/s)
        sim_pos; %1x2 array describing current world-referenced Create3 position (m)
        sim_vel; %1x2 array describing current world-referenced Create3 velocity (m/s)
        sim_eul; %1x3 array describing current world-referenced Create3 orientation (roll/pitch/yaw, radians)
        sim_u; %scalar describing body-referenced Create3 linear velocity (m/s)
        sim_r; %scalar describing body-referenced Create3 angular velocity (rad/sec)
        sim_vel_prev; %1x2 array describing previous world-referenced Create3 velocity (m/s)
        sim_ort_prev; %scalar describing previous world-referenced yaw angle (rad)
        %pos; % placeholder for MoCap data if used
        %quat; % placeholder for MoCap data if used
        %eul; % placeholder for MoCap data if used
        odom_pos; %1x3 array describing current odometry estimate of Create3 position (m) referenced to initial pose of the Create3
        odom_vel; %1x3 array describing current odometry estimate of Create3 linear velocity (m/s) referenced to the body-fixed frame
        odom_quat; %1x4 array describing the current odometry estimate of the Create3 orientation described as a quaternion (wxyz) referenced to the initial pose of the Create3
        odom_omega; %1x3 array describing current odometry estimate of Create3 angular velocty (rad/s) described as a roll/pitch/yaw rate referenced to the body-fixed frame
        odom_eul; %1x3 array describing the current odometry estimate of the Create3 orientation described in Euler angles (roll/pitch/yaw) referenced to the initial pose of the Create3
        accel; %TODO document me
        gyro; %TODO document me
        imu_quat; %TODO document me
        imu_eul; %TODO document me
        pose_sub; %TODO document me
        imu_sub; %TODO document me
        odom_sub; %TODO document me
        cmd_pub; %TODO document me

        % visualization objects
        hCreate3D; %patch handle for Create3 3D visualization
        hCreate2D; %polygon handle for Create3 2D visualization
        hRoom; %polygon handle defining free and occupied space
        h_g2a; %hgtransform handle defining global frame relative to axes frame
        h_b2g; %hgtransform handle defining Create3 body frame relative to global frame
        h_p2b; %hgtransform handle defining Create3 patch object frame relative to body frame

        % utility properties
        psRoom; %polyshape defining free and occupied space
        psCreate; %simplified polyshape representation of Create3 for collision detection
        psCreate0; %simplified polyshape representation of Create3 for collision detection defined at initial position and orientation
        H_b2g_i; %current pose of robot body frame relative to global frame
        H_b2g_0; %initial pose of robot body frame relative to global frame (reference for odometry)
    end

    % --------------------------------------------------------------------
    % Constructor/Destructor
    % --------------------------------------------------------------------
    methods(Access='public')
        % --- Constructor
        function obj = Create3sim(varargin)
            % CREATE3SIM creates a simulation object and visualization
            % for the Create3.
            %   obj = Create3sim;
            %   obj = Create3sim(initial_position)
            %   obj = Create3sim(initial_orientation)
            %   obj = Create3sim(initial_position, initial_orientation)
            %
            %   Input(s)
            %       initial_position - 2-element array specifying the
            %                          initial position of the Create3 in
            %                          meters.
            %    initial_orientation - scalar value specifying the initial
            %                          orientation of the Create3 in
            %                          radians.
            %
            %   L. DeVries, 06Oct2024, USNA

            % Check/parse input(s)
            narginchk(0,2);

            % Set defaults
            initial_position = [0,0];
            initial_orientation = 0;

            % Parse input(s)
            tfPos = false;
            tfAng = false;
            for i = 1:nargin
                switch numel(varargin{i})
                    case 1
                        if tfAng
                            warning('Angle has already been specified as %.2f.',initial_orientation);
                            continue;
                        end
                        initial_orientation = varargin{i};
                        tfAng = true;
                    case 2
                        if tfPos
                            warning('Position has already been specified as [%.2f, %.2f].',initial_position);
                            continue;
                        end
                        initial_position = varargin{i};
                        tfPos = true;
                    otherwise
                        warning('Position must be specified as a 2-element array and orientation must be a scalar.');
                        continue
                end
            end

            % Initialize timer interrupt to update vehicle state
            % -> Simulation Timer
            obj.tmr_updateSim = timer;
            obj.tmr_updateSim.TimerFcn = @(~,~)obj.updateCallback;
            obj.tmr_updateSim.Period = obj.period_updateSim;
            obj.tmr_updateSim.ExecutionMode = 'fixedSpacing';
            obj.tmr_updateSim.BusyMode = 'drop';%'queue';
            obj.tmr_updateSim.ErrorFcn = @(~,~)disp('Simulation Timer Error!');
            obj.tmr_updateSim.Name = 'Create3sim: Simulation Timer';
            obj.tmr_updateSim.Tag  = 'Create3sim: Simulation Timer';
            % -> Simulation Timer
            obj.tmr_updateVis = timer;
            obj.tmr_updateVis.TimerFcn = @(~,~)obj.plotCallback;%, @(~,~)obj.imuCallback};%, @obj.poseCallback, @obj.odomCallback};
            obj.tmr_updateVis.Period = obj.period_updateVis;
            obj.tmr_updateVis.ExecutionMode = 'fixedSpacing';
            obj.tmr_updateVis.BusyMode = 'drop';%'queue';
            obj.tmr_updateVis.ErrorFcn = @(~,~)disp('Visualization Timer Error!');
            obj.tmr_updateVis.Name = 'Create3sim: Visualization Timer';
            obj.tmr_updateVis.Tag  = 'Create3sim: Visualization Timer';

            % Initialize ROS 2 subscribe/publish connections
            obj.pose_sub = 'Simulation mode: no connection to ROS 2 network';
            obj.imu_sub  = 'Simulation mode: no connection to ROS 2 network';
            obj.odom_sub = 'Simulation mode: no connection to ROS 2 network';
            obj.cmd_pub  = 'Simulation mode: no connection to ROS 2 network';

            % Initialize simulation parameters
            obj.sim_pos(:,1) = initial_position;
            obj.sim_vel = [0; 0];
            obj.sim_eul = [0; 0; initial_orientation];
            obj.sim_u = 0;
            obj.sim_r = 0;
            obj.t_now = 0;
            obj.t_last_input = 0;

            % Initialize starting pose
            obj.H_b2g_0 = ...
                Tx(initial_position(1))*Ty(initial_position(2))*...
                Rz(initial_orientation);

            % Initialize current pose
            obj.H_b2g_i = obj.H_b2g_0;

            % Create simulation figure & handles
            obj.fig = figure('Name','Create3 Simulation',...
                'CloseRequestFcn',@(src,event)delete(obj),'MenuBar','none',...
                'NumberTitle','off','Tag','Create3sim: Figure');
            obj.axs = axes('Parent',obj.fig,'NextPlot','add',...
                'DataAspectRatio',[1 1 1],'ZDir','Reverse','YDir','Reverse',...
                'Tag','Create3sim: Axes');
            lgt = addSingleLight(obj.axs);
            set(lgt,'Position',[1,0,-1]);

            % Load Create3
            figTMP = open('Create3_Visualization.fig');
            obj.hCreate3D = findobj(figTMP,'Type','Patch');

            % Create simulation transformation handles
            H_p2b = Rx(pi);
            obj.h_g2a = triad('Parent',obj.axs,'Scale',0.5,'LineWidth',1.5);%,...
            %'AxisLabels',{'g_1','g_2','g_3'});
            obj.h_b2g = triad('Parent',obj.h_g2a,'Scale',0.5,'LineWidth',1.5);%,...
            %'AxisLabels',{'b_1','b_2','b_3'});
            obj.h_p2b = triad('Parent',obj.h_b2g,'Scale',0.5,'LineWidth',1.5,...
                'Matrix',H_p2b);%,'AxisLabels',{'c_1','c_2','c_3'});

            % Move Create3 visualization into simulation
            set(obj.hCreate3D,'Parent',obj.h_p2b);
            % Delete temporary Create3 figure
            delete(figTMP);

            % Hide unnecessary frame(s)
            hideTriad(obj.h_p2b);

            % Label axes
            xlabel(obj.axs,'x (m)');
            ylabel(obj.axs,'y (m)');

            % Define Create3 collision polyshape
            % -> Get patch-referenced vertices
            X_p = obj.hCreate3D.Vertices.';
            X_p(4,:) = 1;
            % -> Reference vertices to body frame
            X_b = H_p2b*X_p;
            % -> Isolate x/y vertices
            X_b = X_b(1:2,:);
            % -> Calculate convex hull
            k = convhull(X_b(1,:),X_b(2,:));
            k(end) = [];
            % -> Keep convex hull of vertices
            X_b = X_b(:,k);
            % -> Create polyshape
            obj.psCreate = polyshape(X_b(1,:),X_b(2,:));
            obj.psCreate0 = obj.psCreate;

            % Plot Create3 2D collision polyshape
            obj.hCreate2D = plot(obj.axs,obj.psCreate,'FaceColor','k',...
                'FaceAlpha',0.5,'EdgeColor','k');

            % Update hRoom
            obj.updateRoom('none');

            % Update
            obj.updateCallback;

            % Update Create3 visualization
            obj.plotCallback();
            drawnow;

            % Shutdown old simulations
            obj.deleteOpenSims();

            % Bring simulation figure to the foreground
            figure(obj.fig);
            drawnow;

            % Start timer
            % -> Simulation
            obj.tmr_updateSim.start;
            fprintf('Starting simulation...');
            pause( obj.period_updateSim );
            fprintf('[DONE]\n');
            % -> Visualization
            obj.tmr_updateVis.start;
            fprintf('Starting visualization...');
            pause( obj.period_updateVis );
            fprintf('[DONE]\n');
        end

        % --- Destructor
        function delete(obj)
            fprintf('Destructor Called.\n')
            fprintf('\tStopping simulation...');
            obj.tmr_updateSim.stop;
            fprintf('[DONE]\n');
            fprintf('\tStopping visualization...');
            obj.tmr_updateVis.stop;
            fprintf('[DONE]\n');
            fprintf('\tDeleting objects...');
            delete(obj.tmr_updateSim);
            delete(obj.tmr_updateVis);
            delete(obj.fig);
            fprintf('[DONE]\n')
        end
    end

    % --------------------------------------------------------------------
    % Private methods
    % --------------------------------------------------------------------
    methods(Access='public',Hidden=true)
        function updateCallback(obj)
            % UPDATECALLBACK updates the simulation evolution and simulated
            % sensor outputs

            % Update current time
            obj.t_now = obj.t_now + obj.period_updateSim;

            % Update previous velocity and heading
            obj.sim_vel_prev = obj.sim_vel;
            obj.sim_ort_prev = obj.sim_eul(3);

            % Check if speed and angular rate commands have expired
            % -> set to command inputs to zero if they have
            if (obj.t_now - obj.t_last_input) > obj.t_timeout
                obj.sim_u = 0;
                obj.sim_r = 0;
            end

            % Update position and orientation of Create3
            obj.sim_vel = [...
                obj.sim_u*cos(obj.sim_eul(3));...
                obj.sim_u*sin(obj.sim_eul(3))];
            obj.sim_pos = obj.sim_pos + obj.sim_vel*obj.period_updateSim;
            obj.sim_eul = obj.sim_eul + [0;0;obj.sim_r]*obj.period_updateSim;

            % Update Create3 pose
            obj.H_b2g_i = ...
                Tx(obj.sim_pos(1))*Ty(obj.sim_pos(2))*Rz(obj.sim_eul(3));

            % Update sensor feedback
            obj.odomCallback();
            obj.imuCallback();
        end

        function plotCallback(obj)
            % PLOTCALLBACK updates the visualization of the Create3

            % Update 3D visualization
            set(obj.h_b2g,'Matrix',obj.H_b2g_i);

            % Update 2D visualization
            if obj.checkCollosions
                psTMP = obj.psCreate0;
                psTMP = rotate(psTMP,rad2deg(obj.sim_eul(3)));
                psTMP = translate(psTMP,obj.sim_pos(1),obj.sim_pos(2));
                obj.psCreate = psTMP;
                set(obj.hCreate2D,'Shape',obj.psCreate,'Visible','on');

                % Highlight collisions
                if obj.isCollided
                    set(obj.hCreate3D,'FaceColor',[255, 0, 0]./255);
                else
                    set(obj.hCreate3D,'FaceColor',[246, 246, 243]./255);
                end
            else
                set(obj.hCreate2D,'Visible','off');
            end

            % Allow plot to update
            drawnow;
        end

        function imuCallback(obj)
            % IMUCALLBACK emulates noisy IMU data
            % emulates form of ROS 2 hardware implementation

            % Accelerometer info
            obj.accel(1) = sign(obj.sim_u)*...
                norm((obj.sim_vel - obj.sim_vel_prev)/obj.period_updateSim);
            obj.accel(2) = randn;
            obj.accel(3) = randn;

            % Gyro info
            obj.gyro(1) = randn;
            obj.gyro(2) = randn;
            obj.gyro(3) = (obj.sim_eul(3)-obj.sim_ort_prev)/obj.period_updateSim;

            %obj.imu_quat(1) = msg.orientation.w;
            %obj.imu_quat(2) = msg.orientation.x;
            %obj.imu_quat(3) = msg.orientation.y;
            %obj.imu_quat(4) = msg.orientation.z;
            obj.imu_eul = obj.sim_eul;
        end

        function odomRosCallback(obj,msg)
            % emulates form of ROS 2 hardware implementation
            obj.odom_pos(1) = msg.pose.pose.position.x;
            obj.odom_pos(2) = msg.pose.pose.position.y;
            obj.odom_pos(3) = msg.pose.pose.position.z;

            obj.odom_quat(1) = msg.pose.pose.orientation.w;
            obj.odom_quat(2) = msg.pose.pose.orientation.x;
            obj.odom_quat(3) = msg.pose.pose.orientation.y;
            obj.odom_quat(4) = msg.pose.pose.orientation.z;
            obj.odom_eul = quat2eul(obj.odom_quat,'XYZ');

            obj.odom_vel(1) = msg.twist.twist.linear.x;
            obj.odom_vel(2) = msg.twist.twist.linear.y;
            obj.odom_vel(3) = msg.twist.twist.linear.z;

            obj.odom_omega(1) = msg.twist.twist.angular.x;
            obj.odom_omega(2) = msg.twist.twist.angular.y;
            obj.odom_omega(3) = msg.twist.twist.angular.z;
        end

        function odomCallback(obj)
            % ODOMCALLBACK simulates Create3 odometry information and
            % emulates the form of ROS 2 hardware implementation

            % Define relative pose
            H_b2b0 = invSE(obj.H_b2g_0)*obj.H_b2g_i;
            R_b2b0 = H_b2b0(1:3,1:3);
            d_b2b0 = H_b2b0(1:3,4);

            % Update robot position estimate
            obj.odom_pos = d_b2b0.';

            % Update orientation
            obj.odom_quat = rotm2quat(R_b2b0);
            obj.odom_eul = quat2eul(obj.odom_quat,'XYZ');

            % TODO - confirm odom_vel
            % -> Is this referenced to the body frame or starting frame?
            %obj.odom_vel = ( R_b2b0*[obj.sim_u; 0; 0] ).'; % start frame referenced
            obj.odom_vel = [obj.sim_u, 0, 0]; % body frame referenced

            % TODO - confirm odom_omega
            % -> Is this referenced to the body frame or starting frame?
            %obj.odom_omega = ( R_b2b0*[0; 0; obj.sim_r] ).'; % start frame referenced
            obj.odom_omega = [0, 0, obj.sim_r]; % body frame referenced
        end

        function deleteOpenSims(obj)
            % DELETEOPENSIMS deletes any existing simulation timers

            % Find existing simulation figures
            fig_TMP = findall(0,'Type','Figure','Tag',obj.fig.Tag);
            tf = fig_TMP ~= obj.fig;
            if nnz(tf)
                fprintf('Open Create3 simulations found: %d\n',nnz(tf));
                fprintf('\tDuplicating figures...')
                fig_CPY = copyobj(fig_TMP(tf),0);
                set(fig_CPY,'CloseRequestFcn','closereq',...
                    'MenuBar','figure','NumberTitle','on',...
                    'Tag','OLD Create3sim: Figure',...
                    'Name','');
                fprintf('[DONE]\n');
                str = sprintf('\n%s  Closing open simulations  %s',...
                    repmat('v',1,10),repmat('v',1,10));
                fprintf('%s\n',str);
                for i = find(tf)
                    fprintf('OLD SIMULATION %d: ',i)
                    close(fig_TMP(i));
                end
                fprintf('%s\n\n',repmat('^',1,numel(str)));
                drawnow;
            end

        end

        function setPose(obj,x,y,psi)
            % SETPOSITION sets the pose (position and orientation) of the
            % Create3.
            %   setPose(obj,x,y,psi)
            %
            %   Input(s)
            %       x - scalar world-referenced x-position (meters)
            %       y - scalar world-referenced y-position (meters)
            %     psi - scalar world-referenced orientation (radians)
            %
            %   M. Kutzer, 22Oct2024, USNA

            % Update position and orientation of Create3
            obj.sim_vel = [0; 0];
            obj.sim_pos = [x; y];
            obj.sim_eul = [0; 0; psi];

            % Update Create3 pose
            obj.H_b2g_i = ...
                Tx(obj.sim_pos(1))*Ty(obj.sim_pos(2))*Rz(obj.sim_eul(3));

            % Reset original pose
            obj.H_b2g_0 = obj.H_b2g_i;

            % Update plot
            obj.plotCallback();
        end
    end

    % --------------------------------------------------------------------
    % Public methods
    % --------------------------------------------------------------------
    methods(Access='public')
        function [pose,vel] = getOdomPose(obj,varargin)
            % GETODOMPOSE gets the current odometry pose information
            %   [pose,vel] = obj.getOdomPose()
            %   ___ = obj.getOdomPose(ref)
            %
            %   Input(s)
            %       ref - character array defining reference for pose
            %             information {['Initial'],'World'}
            %   Output(s)
            %       pose - 1x6 array [x,y,z,roll,pitch,yaw] defining
            %              position and orientation estimates of the
            %              Create3 defined using odometry information
            %              relative to the Create3's initial pose.
            %        vel - 1x6 array defining the linear and angular
            %              velocity estimates for the Create3 defined using
            %              odometry information relative to the current
            %              body-fixed frame.
            %              [x_dot,y_dot,z_dot,roll_dot,pitch_dot,yaw_dot]
            %
            %   M. Kutzer & L. DeVries, 16Oct2024, USNA

            if nargin > 1
                ref = varargin{1};
            else
                ref = 'initial';
            end

            switch lower( ref )
                case 'initial'
                    pose = [obj.odom_pos, obj.odom_eul];
                case 'world'
                    H_b2g = obj.H_b2g_i;
                    d_b2g = H_b2g(1:3,4);
                    R_b2g = H_b2g(1:3,1:3);

                    quat = rotm2quat(R_b2g);
                    eul = quat2eul(quat,'XYZ');

                    % Update robot position estimate
                    pose = [d_b2g.', eul];
                otherwise
                    error('Unrecognized reference');
            end

            % Get pose and velocity odometry
            vel =  [obj.odom_vel, obj.odom_omega];

            % Account for empty values
            if numel(pose) ~= 6
                fprintf('*.getOdomPose: No odometry position/orientation available. Returning zeros.\n');
                pose = zeros(1,6);
            end
            if numel(vel) ~= 6
                fprintf('*.getOdomPose: No odometry linear/angular velocity available. Returning zeros.\n');
                vel = zeros(1,6);
            end

        end

        function [imu_data,imu_eul] = getImuData(obj)
            % GETIMUDATA gets the current IMU information from the Create3
            % simulation.
            %   [imu_data,imu_eul] = obj.getImuData()
            %
            %   Inputs
            %
            %   Outputs
            %       imu_data - 1x6 array defining accelerometer and gyro
            %                  data [a1,a2,a3,g1,g2,g3]
            %        imu_eul - 1x3 array defining [roll,pitch,yaw] in
            %                  radians
            %
            %   L. DeVries, 08Oct2024, USNA

            % Get imu data
            imu_data = [obj.accel, obj.gyro];
            %quat_data = obj.imu_quat;
            imu_eul = obj.imu_eul; % quat2eul(obj.odom_quat,'XYZ');

            % Account for empty values
            if numel(imu_data) ~= 6
                fprintf('*.getImuData: No accelerometer or gyro data available. Returning zeros.\n');
                imu_data = zeros(1,6);
            end
            if numel(imu_eul) ~= 3
                fprintf('*.getImuData: No Euler angle data available. Returning zeros.\n');
                imu_eul = zeros(1,3);
            end

        end

        function setVelCmd(obj,u,r)
            % SETVELCMD sets the linear and angular velocities of the
            % Create3 simulation
            %   obj.setVelCmd(u,r)
            %
            %   Input(s)
            %       u - scalar value defining forward velocity or speed (m/s)
            %       r - scalar value angular velocity or angular rate (rad/s)
            %
            %   L. DeVries, 07Oct2024, USNA

            % Check input(s)
            narginchk(3,3);

            if numel(u) ~= 1
                error('Forward velocity "u" must be a scalar value.');
            end
            if numel(r) ~= 1
                error('Turn rate "r" must be a scalar value.');
            end
            
            % Cap speeds
            if abs(u) > obj.maxLinearSpeed
                u = sign(u)*obj.maxLinearSpeed;
            end
            if abs(r) > obj.maxAngularSpeed
                r = sign(r)*obj.maxAngularSpeed;
            end
            
            % Limit command frequency
            obj.t_last_call = fliplr(obj.t_last_call);
            obj.t_last_call(2) = datetime('now');
            dt = seconds( diff(obj.t_last_call) );
            if dt < obj.period_updateSim
                % Warn user
                dt_pause = ceil( (obj.period_updateSim - dt)*10000 )/10000;
                str = sprintf([...
                    'INPUT IGNORED\n',...
                    '\tCommand frequency of %.3f s is faster than the simulation update period_updateSim of %.3f s.\n' ...
                    '\tConsider adding "pause(%.3f)" between subsequent commands.'],...
                    dt,obj.period_updateSim,dt_pause);
                warning(str);

                % Force pause
                pause(dt_pause);
                return
            end

            % Update time of last input
            obj.t_last_input = obj.t_now;

            % Update properties for commanded speed and angular rate
            obj.sim_u = u;
            obj.sim_r = r;
        end

        function updateRoom(obj,varargin)
            % UPDATEROOM updates the hRoom collision space
            %   updateRoom(obj,str)
            %   updateRoom(obj,bin,pix2m,H_c2o)
            %
            %   Input(s)
            %       str - character array specifying pre-defined spaces.
            %             {'HP208'}
            %       bin - MxN binary image (black/false represents
            %             obstacles)
            %     pix2m - scalar value defining the ratio of linear units
            %             per pixel.
            %     H_c2o - 4x4 transformation defining the "upper left
            %             corner" frame of free-space relative to the
            %             desired world/global frame for the returned
            %             polyshape.
            %
            %   L. DeVries & M. Kutzer, 16Oct2024, USNA

            % Check and parse input(s)
            narginchk(2,4);

            switch nargin
                case 2
                    str = varargin{1};
                    switch lower(str)
                        case 'hp208'
                            %im = imread('Occupancy208_100m2p.png');
                            im = imread('Occupancy208_75m2p.png');
                            bin = imbinarize( rgb2gray(im) );
                            %pix2m = 1/100;
                            pix2m = 1/75;
                            H_c2o = Tx(3098.8/1000)*Ty(-5384.8/1000)*Rz(pi/2);
                        case 'none'
                            bin = true;
                            pix2m = 1;
                            H_c2o = eye(4);
                        otherwise
                            warning('"%s" is not a recognized hRoom.',str);
                            return
                    end
                case 3
                    warning('You must specify the hRoom or necessary parameters to define the hRoom.');
                    return
                case 4
                    % Check inputs using try/catch below
            end

            try
                obj.psRoom = bw2polyshape(bin,pix2m,H_c2o);
            catch
                warning('Unable to update hRoom.');
                obj.psRoom = polyshape;
            end

            delete(obj.hRoom);
            obj.hRoom = plot(obj.psRoom,'FaceColor','k','EdgeColor','none',...
                'Parent',obj.axs);
        end

        function resetPosition(obj,varargin)
            % RESETPOSITION resets the position of the vehicle
            %   resetPosition(obj,position,orientation)
            %
            %   Input(s)
            %       position - 2x1 position vector [xpos,ypos]
            %
            %       orientation - scalar orientation defined on [0,2*pi)
            %
            %   L. DeVries & M. Kutzer, 17Oct2024, USNA


            % Check/parse input(s)
            narginchk(1,3);
            if nargin==1 % case where the user provided no input, reset to origin
                position = [0;0];
                orientation = 0;
            else % case where user provided a reset position and/or orientation
                for i = 1:nargin-1
                    switch numel(varargin{i})
                        case 1
                            orientation = varargin{i};
                        case 2
                            position = varargin{i};
                        otherwise
                            warning('Position must be specified as a 2-element array and orientation must be a scalar.');
                            continue
                    end
                end
            end
            obj.sim_pos = reshape(position,2,1);
            obj.sim_eul = [0;0;orientation];
        end

        function resetTimer(obj)
            % RESETTIMER resets the timer object using a stop/start toggle
            %
            %   NOTE: This is for debugging purposes.
            %
            %   M. Kutzer, 21Oct2024, USNA
            obj.setVelCmd(0,0);
            obj.tmr_updateSim.stop;
            pause(obj.period_updateSim);
            obj.tmr_updateSim.start;
        end

        function tf = isCollided(obj)
            % ISCOLLIDED checks if the Create3 has collided with anything
            % in the environment
            %
            %   M. Kutzer, 12Oct2024, USNA

            % Check properties
            if isempty(obj.psRoom) || isempty(obj.psCreate)
                tf = false;
                return
            end

            % Check for collision
            psInt = intersect(obj.psRoom,obj.psCreate);

            if psInt.NumRegions > 0
                tf = true;
            else
                tf = false;
            end
        end

        function updateTimers(obj)
            % UPDATETIMERS updates the simulation timer periods to
            % match the average timer periods.
            %
            %   M. Kutzer, 22Oct2024, USNA

            fprintf('Simulation Timer:\n')
            fprintf('\tCurrent Period: %6.4f\n',obj.tmr_updateSim.Period);
            fprintf('\tAverage Period: %6.4f\n',obj.tmr_updateSim.AveragePeriod);
            obj.period_updateSim = ceil(obj.tmr_updateSim.AveragePeriod*1000)/1000;

            fprintf('Visualization Timer:\n')
            fprintf('\tCurrent Period: %6.4f\n',obj.tmr_updateVis.Period);
            fprintf('\tAverage Period: %6.4f\n',obj.tmr_updateVis.AveragePeriod);
            obj.period_updateVis = ceil(obj.tmr_updateVis.AveragePeriod*1000)/1000;

            fprintf('Stopping timers...');
            obj.tmr_updateSim.stop;
            obj.tmr_updateVis.stop;
            fprintf('[DONE]\n');

            fprintf('Updating periods...\n');
            obj.tmr_updateSim.Period = obj.period_updateSim;
            fprintf('\t   Simulation Period: %6.4f [DONE]\n',obj.tmr_updateSim.Period);
            obj.tmr_updateVis.Period = obj.period_updateVis;
            fprintf('\tVisualization Period: %6.4f [DONE]\n',obj.tmr_updateVis.Period);

            fprintf('Starting timers...');
            obj.tmr_updateSim.start;
            obj.tmr_updateVis.start;
            fprintf('[DONE]\n');
        end

    end

end