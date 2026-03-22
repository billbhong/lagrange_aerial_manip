% generateBodyRatesFunction.m
% Run this script to generate the numerical function for omega_b

clear; clc;

%% 1. Define Symbolic Variables
syms p_dot_e1 p_dot_e2 p_dot_e3 real
syms p_ddot_e1 p_ddot_e2 p_ddot_e3 real % 2nd derivative of linear momentum
syms psi psi_dot m_t g real

p_dot_e = [p_dot_e1; p_dot_e2; p_dot_e3];
p_ddot_e = [p_ddot_e1; p_ddot_e2; p_ddot_e3];

%% 2. Get thrust, phi, and theta
[T, phi, theta] = computeThrustAndAttitude(p_dot_e, psi, m_t, g);

%% 3. Compute Time Derivatives using the Chain Rule (Jacobians)
% By chain rule: d(phi)/dt = (d(phi)/d(p_dot_e)) * p_ddot_e + (d(phi)/d(psi)) * psi_dot

phi_dot = jacobian(phi, p_dot_e) * p_ddot_e + jacobian(phi, psi) * psi_dot;
theta_dot = jacobian(theta, p_dot_e) * p_ddot_e + jacobian(theta, psi) * psi_dot;

% Assemble the Euler angle rates vector
xi_dot = [phi_dot; theta_dot; psi_dot];

%% 4. Map Euler Rates to Body Rates (omega_b)
% In beginning of paper
Xi = [1,          0,               -sin(theta);
      0,          cos(phi),         sin(phi)*cos(theta);
      0,         -sin(phi),         cos(phi)*cos(theta)];

omega_b = Xi * xi_dot;

%% 5. Auto-Generate the Numerical Function
% Create 'computeBodyRates.m'
disp('Generating computeBodyRates.m...');
matlabFunction(omega_b, ...
    'File', 'computeBodyRates', ...
    'Vars', {p_dot_e, p_ddot_e, psi, psi_dot, m_t, g}, ...
    'Optimize', true); % 'Optimize' greatly simplifies the generated math
disp('Done!');