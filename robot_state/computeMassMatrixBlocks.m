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
    % For a rigidBodyTree with a 6-DOF floating base, the configuration vector
    % typically expects: [Base X, Y, Z, Base Quat W, X, Y, Z, Joint 1 ... Joint k]
    % Because the mass matrix blocks we need are invariant to the base's global 
    % position and orientation, we can safely set the base pose to zero/identity.
    base_pos = [0; 0; 0];
    base_quat = [1; 0; 0; 0]; % Identity quaternion (no rotation)
    
    config = [base_pos; base_quat; eta]; 
    
    % 2. Calculate the full system mass matrix H
    % H will be a symmetric matrix of size (6+k) x (6+k)
    % The first 6 rows/cols represent the 6-DOF floating base (Linear then Angular).
    % The remaining 'k' rows/cols represent the manipulator joints.
    H = massMatrix(robot, config);
    
    % 3. Partition the Matrix
    % H is structured as:
    % [ M_v      M_pw     M_pl ]  <- Base Linear Dynamics (Rows 1:3)
    % [ M_pw^T   M_w      M_wl ]  <- Base Angular Dynamics (Rows 4:6)
    % [ M_pl^T   M_wl^T   M_l  ]  <- Manipulator Dynamics (Rows 7:end)
    
    % Extract the specific coupling blocks needed for the flatness transform
    M_blocks.M_pw = H(1:3, 4:6);    % 3x3: Linear-Angular coupling
    M_blocks.M_pl = H(1:3, 7:end);  % 3xk: Linear-Joint coupling
    M_blocks.M_w  = H(4:6, 4:6);    % 3x3: Base Angular inertia
    M_blocks.M_wl = H(4:6, 7:end);  % 3xk: Angular-Joint coupling
    
end