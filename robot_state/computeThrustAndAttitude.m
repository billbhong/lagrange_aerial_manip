function [T, phi, theta] = computeThrustAndAttitude(p_dot_e, psi, m_t, g)
    % COMPUTETHRUSTANDATTITUDE Calculates total thrust, roll, and pitch
    % mapped from the derivative of the linear momentum and the yaw angle.
    %
    % Inputs:
    %   p_dot_e : 3x1 column vector, derivative of linear momentum in inertial frame
    %   psi     : scalar, desired yaw angle (radians)
    %   m_t     : scalar, total mass of the multi-rotor and manipulator system
    %
    % Outputs:
    %   T       : scalar, total thrust
    %   phi     : scalar, roll angle (radians)
    %   theta   : scalar, pitch angle (radians)

    % Standard basis vectors for the inertial frame
    e1 = [1; 0; 0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];

    %% Step 1.1: Calculate Total Thrust (T)
    % Equation: T = || p_dot_e + m_t * g * e3 ||_2
    T_vec = p_dot_e + (m_t * g) * e3;
    T = norm(T_vec); 
    
    %% Step 1.2: Calculate Roll (phi)
    % Equation: phi = asin( (e1^T * p_dot_e * sin(psi) - e2^T * p_dot_e * cos(psi)) / T )
    
    phi_numerator = e1' * p_dot_e * sin(psi) - e2' * p_dot_e * cos(psi);
    phi = asin(phi_numerator / T);
    
    %% Step 1.3: Calculate Pitch (theta)
    % Equation: theta = atan( (e1^T * p_dot_e * cos(psi) + e2^T * p_dot_e * sin(psi)) / (e3^T * p_dot_e + m_t * g) )
    
    theta_numerator = e1' * p_dot_e * cos(psi) + e2' * p_dot_e * sin(psi);
    theta_denominator = e3' * p_dot_e + m_t * g;

    theta = atan2(theta_numerator, theta_denominator);
    
end