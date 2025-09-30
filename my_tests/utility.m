classdef utility
    % Usage from main.m, e.g.:
    %   waypoints_R = utility.transform_A_to_R(waypoints_A, dims, limits, waypoint_e);
    %   traj        = utility.linear_interpolator(waypoints_R);
    %   utility.perform_experiment(N, control_params, exp_time, traj, img, limits, img_e);

    methods(Static)
        % -----------------------------------------------------------------
        % Transform waypoints from Neural ATTF frame to Robotarium frame
        function w_R = transform_A_to_R(w_A, dims, limits, waypoint_e)
            alpha =  (2*limits(1))/dims(1);
            beta  =  (2*limits(2))/dims(2);
            M = [alpha, 0; 0, beta];
            offset = [limits(1)-waypoint_e(1); limits(2)];
            w_R = zeros(size(w_A));
            for k = 1:size(w_A, 3)
                temp = M * w_A(:, :, k);
                for j = 1:size(w_A, 2)
                    w_R(:, j, k) = temp(:, j) - offset;
                end
            end
        end

        % -----------------------------------------------------------------
        % Interpolate trajectory at 30Hz between given 1Hz waypoints.
        function trajectory = linear_interpolator(waypoints)
            f = 30;
            [h, w, l] = size(waypoints);
            trajectory = zeros(h, w, 1+(l-1)*f);
            for k = 1:l-1
                ds = (waypoints(:, :, k+1) - waypoints(:, :, k))/f;
                for i = 1:f
                    trajectory(:, :, f*(k-1)+i) = waypoints(:, :, k) + (i-1)*ds;
                end
            end
            trajectory(:, :, end) = waypoints(:, :, end);
        end

        % -----------------------------------------------------------------
        % Perform Robotarium experiment simulation
        function perform_experiment(N, control_params, exp_time, traj, img, limits, img_e)
            % Initiating a Robotarium object
            r = Robotarium('NumberOfRobots', N, 'ShowFigure', 'true');

            % Set the background image
            xImg = linspace(-limits(1)-img_e(1), limits(1)-img_e(1), size(img,2));
            yImg = linspace(-limits(2)+img_e(2), limits(2)+img_e(2), size(img,1));
            image(xImg, yImg, flipud(img), 'CDataMapping','scaled');
            set(gca,'YDir','normal');
            axis([-limits(1) limits(1) -limits(2) limits(2)]);
            axis equal; hold on;

            % Initiating per-robot path trails (last 100 iterations)
            seg_len = 100;
            trail = utility.create_trail_manager(N, seg_len, gca);

            % Controller / barrier / dynamics
            si_vel_gains       = control_params(1:2);
            uni_lin_vel_gain   = control_params(3);
            uni_ang_vel_limit  = control_params(4);
            safe_r             = control_params(5);

            si_pos_K = create_si_position_controller( ...
                'XVelocityGain', si_vel_gains(1), ...
                'YVelocityGain', si_vel_gains(2));
            si_barrier_certificate = create_si_barrier_certificate2( ...
                'UnsafeBarrierGain', 1e6, 'SafeBarrierGain', 100, ...
                'SafetyRadius', safe_r);
            si_to_uni_dyn = create_si_to_uni_dynamics( ...
                'LinearVelocityGain', uni_lin_vel_gain,...
                'AngularVelocityLimit', uni_ang_vel_limit);

            % (If needed by your stack)
            % args = {'PositionError', 0.1, 'RotationError', 100};
            % init_checker = create_is_initialized(args{:});

            % Main loop
            iterations = (exp_time*30) + 10;
            dx = zeros(2, N);
            for t = 1:iterations
                % Get current poses
                x = r.get_poses();

                idx = min(t, size(traj, 3));
                x_goal = traj(:, :, idx);

                % SI control
                dx = si_pos_K(x(1:2, :), x_goal);

                % Barrier & map to unicycle
                dx = si_barrier_certificate(dx, x);
                dx = si_to_uni_dyn(dx, x);

                % Clamp controls
                v_max = r.max_linear_velocity;
                w_max = r.max_angular_velocity;
                for i = 1:N
                    if norm(dx(1,i)) > v_max, dx(1,i) = dx(1,i)/norm(dx(1,i))*v_max; end
                    if norm(dx(2,i)) > w_max, dx(2,i) = dx(2,i)/norm(dx(2,i))*w_max; end
                end

                % Apply
                r.set_velocities(1:N, dx);
                r.step();

                % Trails (2xN positions)
                trail = utility.update_trails(trail, x(1:2,:));
            end

            r.debug();
        end
    end

    methods(Static, Access=private)
        % -----------------------------------------------------------------
        % Create N colored polyline handles and ring buffers for trails
        function trail = create_trail_manager(N, seg_len, ax)
            if nargin < 3 || isempty(ax), ax = gca; end
            trail.segLen = seg_len;
            trail.histX  = nan(seg_len+1, N);
            trail.histY  = nan(seg_len+1, N);
            trail.hSeg   = gobjects(N,1);

            cmap = lines(N);
            hold(ax,'on');
            for k = 1:N
                trail.hSeg(k) = line(ax, nan, nan, 'LineWidth', 2, ...
                    'Color', cmap(k,:), 'Clipping','on', 'HitTest','off');
            end
        end

        % -----------------------------------------------------------------
        % Append current positions and redraw polylines
        function trail = update_trails(trail, x_y)
            % Shift ring buffer up and append newest positions at the end
            trail.histX(1:end-1,:) = trail.histX(2:end,:);
            trail.histY(1:end-1,:) = trail.histY(2:end,:);
            trail.histX(end,:)     = x_y(1,:);
            trail.histY(end,:)     = x_y(2,:);

            % Draw per-robot polyline of available history (up to segLen)
            for k = 1:numel(trail.hSeg)
                firstValid = find(~isnan(trail.histX(:,k)), 1, 'first');
                if isempty(firstValid), continue; end
                xpath = trail.histX(firstValid:end, k);
                ypath = trail.histY(firstValid:end, k);
                valid = ~isnan(xpath) & ~isnan(ypath);
                set(trail.hSeg(k), 'XData', xpath(valid), 'YData', ypath(valid));
            end
        end
    end
end