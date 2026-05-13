function ds_b = base_velocity_dynamics(t, s_b, traj_data, aerial_robot, m_t)
    % ODE RHS for the aerial manipulator base position.
    %
    % Pipeline (paper eqs. 21-23 and the equation immediately following):
    %   p_dot, p_ddot, p_dddot, psi, psi_dot, eta, eta_dot
    %     -> p_e, p_dot_e, p_ddot_e     (CoM momentum from flat outputs)
    %     -> T, phi, theta              (computeThrustAndAttitude)
    %     -> omega_b                    (computeBodyRates)
    %     -> p_body = R_eb' * p_e
    %     -> s_b_dot_body = (1/m_t)(p_body - M_pw*omega_b - M_pl*eta_dot)
    %     -> ds_b = R_eb * s_b_dot_body (inertial frame for integration)
    %
    % Inputs:
    %   t            : scalar, current ODE time
    %   s_b          : 3x1, inertial base position (unused inside RHS; this
    %                  pipeline is open-loop on the flat trajectory)
    %   traj_data    : struct with fields t, p, p_dot, p_ddot, p_dddot,
    %                  psi, psi_dot, eta, eta_dot
    %   aerial_robot : rigidBodyTree of the multi-rotor + arm
    %   m_t          : scalar total system mass
    %
    % Output:
    %   ds_b         : 3x1 inertial base velocity at time t

    g = 9.81;

    % --- Interpolate flat outputs at time t ---
    p_dot   = interp1(traj_data.t, traj_data.p_dot',   t)';
    p_ddot  = interp1(traj_data.t, traj_data.p_ddot',  t)';
    p_dddot = interp1(traj_data.t, traj_data.p_dddot', t)';
    psi     = interp1(traj_data.t, traj_data.psi',     t)';
    psi_dot = interp1(traj_data.t, traj_data.psi_dot', t)';
    eta     = interp1(traj_data.t, traj_data.eta',     t)';
    eta_dot = interp1(traj_data.t, traj_data.eta_dot', t)';

    % --- CoM momentum from flat outputs (p_e = m_t * v_CoM) ---
    p_e      = m_t * p_dot;
    p_dot_e  = m_t * p_ddot;
    p_ddot_e = m_t * p_dddot;

    % --- Recover roll, pitch from p_dot_e, psi (eqs. 21-23) ---
    [~, phi, theta] = computeThrustAndAttitude(p_dot_e, psi, m_t, g);

    % --- Body rates omega_b ---
    omega_b = computeBodyRates(p_dot_e, p_ddot_e, psi, psi_dot, m_t, g);

    % --- Mass matrix partitions at current joint angles ---
    M_blocks = computeMassMatrixBlocks(aerial_robot, eta);

    % --- Base velocity in body frame, then rotate to inertial for integration (equation under equation 23)---
    R_eb         = eul2rotm([psi, theta, phi], 'ZYX');
    p_body       = R_eb' * p_e;
    s_b_dot_body = (1/m_t) * (p_body - M_blocks.M_pw * omega_b ...
                                    - M_blocks.M_pl * eta_dot);

    ds_b = R_eb * s_b_dot_body;
end
