function M_blocks = computeMassMatrixBlocks(robot, eta)
    % Extracts the partitioned mass matrix blocks for an aerial manipulator.
    %
    % Inputs:
    %   robot : A rigidBodyTree object modeling the floating base + arm
    %   eta   : Current manipulator joint angles (k x 1 vector)
    %
    % Outputs:
    %   M_blocks : Struct containing the partitioned 3x3 and 3xk blocks

    % 1. Create the configuration vector
    % For a rigidBodyTree with a 6-DOF floating base in column DataFormat,
    % MATLAB expects: [Quat W, X, Y, Z, Base X, Y, Z, Joint 1 ... Joint k]
    % (quaternion first, then translation; see MathWorks "Plan Path of Robotic
    % Arm Mounted on Quadrotor" example).
    base_quat = [1; 0; 0; 0]; % Identity quaternion (no rotation)
    base_pos  = [0; 0; 0];

    config = [base_quat; base_pos; eta];

    % 2. Calculate the full system mass matrix H
    % H is symmetric, size (6+k) x (6+k). MATLAB orders the floating-base
    % velocities as [omega_b; v_b] (angular first, then linear), both in the
    % body-fixed frame. The remaining 'k' rows/cols are the manipulator joints.
    H = massMatrix(robot, config);

    % 3. Partition the Matrix
    % With MATLAB's angular-first floating-base ordering, H is structured as:
    % [ M_w       M_pw^T    M_wl ]  <- Base Angular Dynamics (Rows 1:3)
    % [ M_pw      M_p       M_pl ]  <- Base Linear  Dynamics (Rows 4:6)
    % [ M_wl^T    M_pl^T    M_l  ]  <- Manipulator Dynamics  (Rows 7:end)
    %
    % These M_* match the paper's notation (Eq. 12 of Wei et al. 2021), where
    % the kinetic energy is K = (1/2)[s_b_dot; omega_b; eta_dot]^T M [...].
    M_blocks.M_w  = H(1:3, 1:3);    % 3x3 : Base angular inertia
    M_blocks.M_pw = H(4:6, 1:3);    % 3x3 : Linear-Angular coupling (rows linear, cols angular)
    M_blocks.M_wl = H(1:3, 7:end);  % 3xk : Angular-Joint coupling
    M_blocks.M_p  = H(4:6, 4:6);    % 3x3 : Base linear inertia (should equal m_t * I_3)
    M_blocks.M_pl = H(4:6, 7:end);  % 3xk : Linear-Joint coupling
    M_blocks.M_l  = H(7:end, 7:end);% kxk : Manipulator inertia
    
end