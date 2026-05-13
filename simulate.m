clear; clc;

addpath('robot_state');

k = 2; % Number of manipulator links
link_length = 0.2; % 20cm links
m_t = 0; % weight in kg (added together later)

% Initialize the rigid body tree
aerial_robot = rigidBodyTree('DataFormat', 'column');
aerial_robot.Gravity = [0, 0, -9.81];

%% --- 1. INITIALIZE MULTI-ROTOR BASE WITH VISUALS ---
uav_base = rigidBody('uav_base');
uav_joint = rigidBodyJoint('uav_floating_joint', 'floating');
uav_base.Joint = uav_joint;
uav_base.Mass = 1.5; 
m_t = m_t + uav_base.Mass;
uav_base.Inertia = [0.03 0.03 0.06 0 0 0]; 

% -- Add Visuals to the UAV Base --
% 1. Central Chassis (A small box)
addVisual(uav_base, 'Box', [0.15, 0.15, 0.08], trvec2tform([0, 0, 0]));

% 2. Quadrotor Arms (An 'X' frame using two long, thin boxes)
addVisual(uav_base, 'Box', [0.5, 0.02, 0.02], trvec2tform([0, 0, 0]) * eul2tform([pi/4, 0, 0]));
addVisual(uav_base, 'Box', [0.5, 0.02, 0.02], trvec2tform([0, 0, 0]) * eul2tform([-pi/4, 0, 0]));

% 3. Rotors (Four thin cylinders at the ends of the arms)
rotor_dist = 0.175; % Distance from center to rotors
rotor_height = 0.02;
addVisual(uav_base, 'Cylinder', [0.1, 0.01], trvec2tform([rotor_dist, rotor_dist, rotor_height]));
addVisual(uav_base, 'Cylinder', [0.1, 0.01], trvec2tform([-rotor_dist, rotor_dist, rotor_height]));
addVisual(uav_base, 'Cylinder', [0.1, 0.01], trvec2tform([rotor_dist, -rotor_dist, rotor_height]));
addVisual(uav_base, 'Cylinder', [0.1, 0.01], trvec2tform([-rotor_dist, -rotor_dist, rotor_height]));

addBody(aerial_robot, uav_base, 'base');

%% --- 2. INITIALIZE k-LINKED MANIPULATOR WITH VISUALS ---
parent_body = 'uav_base';

for i = 1:k
    link_name = sprintf('arm_link_%d', i);
    joint_name = sprintf('arm_joint_%d', i);
    
    link = rigidBody(link_name);
    joint = rigidBodyJoint(joint_name, 'revolute');
    
    if i == 1
        % Mount the first joint slightly below the geometric center
        setFixedTransform(joint, trvec2tform([0, 0, -0.05])); 
    else
        % Subsequent joints are placed at the end of the previous link (along X-axis)
        setFixedTransform(joint, trvec2tform([link_length, 0, 0])); 
    end
    
    % Joints pitch up and down (Y-axis rotation)
    joint.JointAxis = [0 1 0]; 
    link.Joint = joint;
    link.Mass = 0.2;
    m_t = m_t + link.Mass;

    % move center of mass to center
    link.CenterOfMass = [link_length/2, 0, 0]; 
    
    % Physically accurate cylinder inertia [Ixx, Iyy, Izz, Iyz, Ixz, Ixy]
    % Using roughly Ixx = 0.00006, Iyy = 0.0007, Izz = 0.0007
    link.Inertia = [0.00006 0.0007 0.0007 0 0 0];
    
    % -- Add Visuals to the Arm Link --
    % MATLAB cylinders are centered at the origin and point along the Z-axis by default.
    % We need to rotate the cylinder so it points along the X-axis, and translate it 
    % so it spans from the current joint to the next joint.
    visual_transform = trvec2tform([link_length/2, 0, 0]) * eul2tform([0, pi/2, 0]);
    addVisual(link, 'Cylinder', [0.025, link_length], visual_transform);
    
    addBody(aerial_robot, link, parent_body);
    parent_body = link_name;
end

%% --- 3. TRAJECTORY GENERATION ---
% Define your time vector (e.g., a 5-second flight at 100Hz)
fs = 60; % Sample rate
t = 0:(1/fs):10; 


% Must match waypoints column size
time_points = [0, 1.5, 3, 5.0, 7, 9, 10];

use_waypoints = false;

if use_waypoints
    % 1. Define start and end poses
    pos_waypoints = [0, 1.5,  3.0;  % X-axis: Flies forward to 3m
                    0, 2.0,  0.5;  % Y-axis: Swings way out to the left (2m), then cuts back
                    0, 1.0,  2.0]; % Z-axis: Climbs to 1m, then up to 2m 
else
    % --- PARAMETRIC HELIX TRAJECTORY ---
    radius = 2.0;            % radius in meters of circle
    omega  = (2*pi) / 3;     % Completes one full circle in /x seconds
    climb_rate = 0.4;        % Climbing velocity
    
    % Position (X, Y, Z)
    p_traj = [radius * sin(omega * t); 
              radius * (1 - cos(omega * t)); % Offset so it starts at Y=0
              climb_rate * t];
    
    % Velocity (Analytical Derivatives of Position)
    p_dot_traj = [radius * omega * cos(omega * t);
                  radius * omega * sin(omega * t);
                  climb_rate * ones(size(t))];
    
    % Acceleration (Analytical Derivatives of Velocity)
    p_ddot_traj = [-radius * omega^2 * sin(omega * t);
                    radius * omega^2 * cos(omega * t);
                    zeros(size(t))];
end

% 2. Define Waypoints for Yaw (psi)
psi_waypoints = [0, -pi, pi/2, -3*pi/4, 0, -pi/2, pi/3];

% 3. Define Waypoints for Joint Angles (eta)
eta_waypoints = [pi/2, 0,  pi,  pi/4, 2*pi/3,       0, pi/2; % first joint
                 0, pi/2, -pi, -pi/2,      0, -2*pi/3,    0]; % 2nd joint

% Generate smooth trajectories (outputs are [values, velocities, accelerations])
if use_waypoints
    [p_traj, p_dot_traj, p_ddot_traj] = quinticpolytraj(pos_waypoints, time_points, t);
end
[psi_traj, psi_dot_traj, ~]       = quinticpolytraj(psi_waypoints, time_points, t);
[eta_traj, eta_dot_traj, ~]       = quinticpolytraj(eta_waypoints, time_points, t);

% Get the jerk needed by numerically finite diff the accel
dt = 1/fs; 
p_dddot_traj = zeros(size(p_ddot_traj));
p_dddot_traj(1,:) = gradient(p_ddot_traj(1,:), dt); % X-axis jerk
p_dddot_traj(2,:) = gradient(p_ddot_traj(2,:), dt); % Y-axis jerk
p_dddot_traj(3,:) = gradient(p_ddot_traj(3,:), dt); % Z-axis jerk

% Calculate Snap
p_ddddot_traj = zeros(size(p_dddot_traj));
p_ddddot_traj(1,:) = gradient(p_dddot_traj(1,:), dt); 
p_ddddot_traj(2,:) = gradient(p_dddot_traj(2,:), dt); 
p_ddddot_traj(3,:) = gradient(p_dddot_traj(3,:), dt); 

% Package trajectory into a struct so the ODE can access it at any time 't'
traj_data.t = t;
% position
traj_data.p = p_traj;
traj_data.p_dot = p_dot_traj;
traj_data.p_ddot = p_ddot_traj;
traj_data.p_dddot = p_dddot_traj;
traj_data.p_ddddot = p_ddddot_traj;
% yaw
traj_data.psi = psi_traj;
traj_data.psi_dot = psi_dot_traj;
% joint angles
traj_data.eta = eta_traj;
traj_data.eta_dot = eta_dot_traj;

%% --- (Disabled) Full closed-loop dynamics with controller ---
% Re-enable once flatness_controller / computeInputsFromFlatOutputs are complete.
%
% q0 = [p_traj(:,1); 0; 0; psi_traj(1); eta_traj(:,1)];
% q_dot0 = zeros(8,1);
% T0 = m_t * 9.81; T_dot0 = 0;
% X0 = [q0; q_dot0; T0; T_dot0];
% options = odeset('RelTol', 1e-3, 'AbsTol', 1e-4);
% [t_out, X_out] = ode45(@(t, X) aerial_manipulator_dynamics(t, X, aerial_robot, traj_data), ...
%                        [t(1), t(end)], X0, options);

%% --- Flatness inversion: integrate base velocity to recover base position ---
% Initial base position: choose s_b(0) so that CoM at t=0 equals p_traj(:,1).
[~, phi0, theta0] = computeThrustAndAttitude(m_t * p_ddot_traj(:,1), psi_traj(1), m_t, 9.81);
R0 = eul2rotm([psi_traj(1), theta0, phi0], 'ZYX');
com_body_0 = centerOfMass(aerial_robot, [0;0;0; 1;0;0;0; eta_traj(:,1)]);
s_b0 = p_traj(:,1) - R0 * com_body_0;

options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
disp('Integrating base velocity via ode45 ...');
[t_out, S_out] = ode45(@(t, s_b) base_velocity_dynamics(t, s_b, traj_data, aerial_robot, m_t), ...
                       [t(1), t(end)], s_b0, options);
%%

% Create a figure window for the animation
figure('Name', 'Aerial Manipulator Simulation', 'Color', 'white', 'Position', [100 100 800 600]);

% FIX 1: Add 'Frames', 'off' to match the loop
ax = show(aerial_robot, 'Frames', 'off'); 

axis(ax, [-5 5 -5 5 0 5]); % Set bounds: [Xmin Xmax Ymin Ymax Zmin Zmax]
view(3); grid on; hold on;
% camlight('headlight'); % Attaches a light source to your camera
% lighting gouraud;      % Turns on smooth 3D shading
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% Plot the reference CoM trajectory (blue) and the integrated base trajectory (red)
traj_line = plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), ...
                  'Color', [0 0.447 0.741], ... % MATLAB standard blue
                  'LineWidth', 2.5);
traj_line.Color(4) = 0.3; % 30% opacity

base_line = plot3(S_out(:,1), S_out(:,2), S_out(:,3), ...
                  'Color', [0.85 0.10 0.10], ... % red
                  'LineWidth', 2.5);
base_line.Color(4) = 0.5; % 50% opacity

legend([traj_line, base_line], {'CoM trajectory (target)', 'Base position (integrated)'}, ...
       'Location', 'northeast', 'AutoUpdate', 'off');

time_hud = annotation('textbox', [0.05, 0.85, 0.2, 0.1], ...
                      'String', 'Time: 0.00 s', ...
                      'FontSize', 14, 'FontWeight', 'bold', ...
                      'LineStyle', 'none', 'Color', 'k');

% --- CHASE CAMERA SETTINGS ---
    % follow_dist = 4.0; % Meters behind the drone
    % cam_height  = 1.0; % Meters above the drone

    offset_x   = -2.5; % 3 meters behind along the X-axis
    offset_y   = 0; % no offset horizontally
    cam_height = 0.7; % above drone

% --- VIDEO RECORDING SETUP ---
    video_filename = 'Aerial_Manipulator_Flight.mp4';
    vid = VideoWriter(video_filename, 'MPEG-4');
    
    % Set playback framerate. If your simulation runs at fs=100, a 100fps video 
    % might be too large or not supported by some players. 30 or 60 is standard.
    vid.FrameRate = 60; 
    vid.Quality = 100; % Max quality
    
    open(vid); % "Turn on" the camera

%% 5. Simulation

for idx = 1:size(t, 2)
    % Grab current values
    current_time_step = t(:, idx);

    % Integrated base position from ode45 (NOT the CoM trajectory directly)
    current_base_pos = interp1(t_out, S_out, current_time_step)';

    current_psi = psi_traj(:, idx);
    current_eta = eta_traj(:, idx);

    % Recover roll and pitch for animation (eqs. 21-23)
    p_dot_e = m_t * p_ddot_traj(:, idx);
    [~, phi, theta] = computeThrustAndAttitude(p_dot_e, current_psi, m_t, 9.81);

    % ZYX (Yaw, Pitch, Roll) -> quaternion [Qw, Qx, Qy, Qz]
    quat = eul2quat([current_psi, theta, phi], 'ZYX');

    % Order for a 'floating' joint in column format is: [Qw; Qx; Qy; Qz; X; Y; Z]
    base_config = [quat'; current_base_pos];

    % Combine base configuration with joint angles
    config = [base_config; current_eta];
    
    % update the visual
    show(aerial_robot, config, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off', 'FastUpdate', true)
    % sgtitle(sprintf('Time: %.2f s', current_time_step), 'FontSize', 14, 'FontWeight', 'bold');
    time_hud.String = sprintf('Time: %.2f s', current_time_step);

      
    % camera look at drone
    camtarget(ax, current_base_pos');

    % for dynamic turning
    % cam_x = current_base_pos(1) - follow_dist * cos(current_psi);
    % cam_y = current_base_pos(2) - follow_dist * sin(current_psi);
    % cam_z = current_base_pos(3) + cam_height;
    %
    % campos(ax, [cam_x, cam_y, cam_z]);

    campos(ax, [current_base_pos(1) + offset_x, current_base_pos(2) + offset_y, current_base_pos(3) + cam_height]);

    camva(ax, 45);
    
    % Optional: Keep the camera's "up" direction stable to prevent rolling
    camup(ax, [0 0 1]);
    
    drawnow limitrate;
    % --- CAPTURE FRAME FOR VIDEO ---
    % getframe(gcf) captures the entire figure window (including titles/HUD)
    frame = getframe(gcf); 
    writeVideo(vid, frame);
    
    % Optional: Uncomment below to make it run closer to real-time 
    % pause(1/fs); 
end

close(vid); % "Turn off" the camera and finalize the file
fprintf('Video successfully saved to: %s\n', video_filename);