function q = computeStateFromFlatOutputs(p_e, p_dot_e, p_ddot_e, psi, psi_dot, eta, eta_dot, m_t, M_blocks)
    % Computes the full state vector q from the flat outputs.
    %
    % Inputs:
    %   p_e, p_dot_e, p_ddot_e : Inertial linear momentum and its derivatives (3x1)
    %   psi, psi_dot           : Yaw angle and rate (scalars)
    %   eta, eta_dot           : Manipulator joint angles and rates (kx1)
    %   m_t                    : Total system mass
    %   M_blocks               : A struct containing the evaluated mass matrix blocks 
    %                            (M_pw, M_pl, M_w, M_wl) for the current 'eta'
    %
    % Output:
    %   q                      : The full state vector 
    
    g = 9.81; % Define gravity (ensure sign matches your plant's convention)

    %% 1. Get Thrust and Attitude
    [T, phi, theta] = computeThrustAndAttitude(p_dot_e, psi, m_t, g);
    
    %% 2. Get Body Rates
    omega_b = computeBodyRates(p_dot_e, p_ddot_e, psi, psi_dot, m_t, g);
    
    %% 3. Calculate Body-Frame Linear Momentum (p)
    % Create Z-Y-X rotation matrix from Euler angles (R_eb
    R_eb = eul2rotm([psi, theta, phi], 'ZYX'); 
    
    p = R_eb' * p_e; % Rotate inertial momentum to body frame
    
    %% 4. Extract Mass Matrix Blocks
    % These must be evaluated dynamically based on the current joint angles (eta)
    M_pw = M_blocks.M_pw;
    M_pl = M_blocks.M_pl;
    M_w  = M_blocks.M_w;
    M_wl = M_blocks.M_wl;
    
    %% 5. Calculate Base Linear Velocity (s_b_dot)
    % Equation: s_b_dot = (1/m_t) * (p - M_pw*omega_b - M_pl*eta_dot)
    s_b_dot = (1/m_t) * (p - M_pw*omega_b - M_pl*eta_dot);
    
    %% 6. Calculate Body-Frame Angular Momentum (l)
    % Equation: l = M_pw^T * s_b_dot + M_w * omega_b + M_wl * eta_dot
    l = M_pw' * s_b_dot + M_w * omega_b + M_wl * eta_dot; 
    
    %% 7. Assemble the Full State Vector (q)
    % q = [p; l; phi; theta; psi; eta; eta_dot]
    q = [p; l; phi; theta; psi; eta; eta_dot];
    
end