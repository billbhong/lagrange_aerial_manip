function [u_de] = flatness_controller(X, flat_des, aerial_robot)
    % NOT USED CURRENTLY
    % 1. UNPACK STATE
    pos = X(1:3);
    vel = X(9:11);
    roll = X(4); 
    pitch = X(5); 
    yaw = X(6);

    roll_dot = X(12); 
    pitch_dot = X(13); 
    yaw_dot = X(14);
    eta = X(7:8);
    eta_dot = X(15:16);
    
    T = X(17);
    T_dot = X(18);
    
    g = 9.81;
    e3 = [0; 0; 1];

    % 2. COMPUTE ROTATION AND ANGULAR VELOCITY
    R = eul2rotm([yaw, pitch, roll], 'ZYX');
    
    % Map Euler rates to Body Angular Velocity (Omega)
    % W maps [roll_dot; pitch_dot; yaw_dot] to body frame [wx; wy; wz]
    W = [1,  0,        -sin(pitch);
         0,  cos(roll), sin(roll)*cos(pitch);
         0, -sin(roll), cos(roll)*cos(pitch)];
    Omega = W * [roll_dot; pitch_dot; yaw_dot];

    % 3. PREPARE ROBOT CONFIGURATION FOR MATLAB KINEMATICS
    quat = eul2quat([yaw, pitch, roll], 'ZYX'); 
    config = [quat'; pos; eta]; % Format for floating base column tree
    
    % To use MATLAB's Jacobian, we need the spatial velocity of the joints
    % Format: [angular_vel_base_frame; linear_vel_base_frame; joint_vels]
    v_base = R' * vel; % Linear velocity in base frame
    v_tree = [Omega; v_base; eta_dot];

    % ==========================================
    % 4. COMPUTE CURRENT FLAT OUTPUTS (p up to Jerk)
    % ==========================================
    
    % Position: Extract total Center of Mass from Rigid Body Tree
    flat_curr.p = centerOfMass(aerial_robot, config);
    
    % Velocity: Use CoM Jacobian
    J_com = centerOfMassJacobian(aerial_robot, config);
    flat_curr.p_dot = J_com * v_tree;
    
    % Acceleration: Newton-Euler shortcut!
    flat_curr.p_ddot = -g*e3 + (T / m_t) * R * e3;
    
    % Jerk: Derivative of Newton-Euler
    flat_curr.p_dddot = (T_dot / m_t) * R * e3 + (T / m_t) * R * cross(Omega, e3);
    
    % Yaw and Joints
    flat_curr.psi     = yaw;
    flat_curr.psi_dot = yaw_dot;
    flat_curr.eta     = eta;
    flat_curr.eta_dot = eta_dot;

    % 2. CLF-QP calculates the commanded highest-order derivatives (virtual control 'v')
    v_cmd = solve_clf_qp(flat_curr, flat_des); 
    
    % 3. USE YOUR FLATNESS INVERSE MAPPING HERE!
    % Instead of plugging in the purely "desired" trajectory, you plug in the 
    % current flat states + the CLF-QP's commanded highest derivatives 
    % to figure out what torques the motors need to apply right now.
    u_de = computeInputsFromFlatOutputs(flat_curr.p, flat_curr.p_dot, ..., v_cmd);
end