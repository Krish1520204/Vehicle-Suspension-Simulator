function modal = validate_model(params)
% VALIDATE_MODEL Performs analytical modal analysis on the 4-DOF half-car model.
%
% Calculates:
%   - Mass matrix (M), Damping matrix (C), Stiffness matrix (K)
%   - Linear state-space system matrix (A_sys)
%   - Undamped natural frequencies (Hz) and mode shapes
%   - Damped eigenvalues, damping ratios (zeta), and damped frequencies (Hz)
%   - Modal classification: Heave, Pitch, Front Wheel Hop, Rear Wheel Hop
%
% Inputs:
%   params - Vehicle parameters struct (from get_vehicle_params)
%
% Output:
%   modal  - Struct containing matrices, frequencies, damping ratios, and summary table

    % Extract parameters
    m_s  = params.m_s;
    I_y  = params.I_y;
    m_uf = params.m_uf;
    m_ur = params.m_ur;

    k_sf = params.k_sf;
    k_sr = params.k_sr;
    k_tf = params.k_tf;
    k_tr = params.k_tr;

    c_sf = params.c_sf;
    c_sr = params.c_sr;
    c_tf = params.c_tf;
    c_tr = params.c_tr;

    a    = params.a;
    b    = params.b;

    % 1. Mass Matrix M (4x4)
    % Coordinates q = [z_s; theta; z_uf; z_ur]
    M = diag([m_s, I_y, m_uf, m_ur]);

    % 2. Stiffness Matrix K (4x4)
    K = [ (k_sf + k_sr),          (k_sf*a - k_sr*b),       -k_sf,             -k_sr;
          (k_sf*a - k_sr*b),      (k_sf*a^2 + k_sr*b^2),   -k_sf*a,            k_sr*b;
         -k_sf,                   -k_sf*a,                 (k_sf + k_tf),      0;
         -k_sr,                    k_sr*b,                  0,                (k_sr + k_tr) ];

    % 3. Damping Matrix C (4x4)
    C = [ (c_sf + c_sr),          (c_sf*a - c_sr*b),       -c_sf,             -c_sr;
          (c_sf*a - c_sr*b),      (c_sf*a^2 + c_sr*b^2),   -c_sf*a,            c_sr*b;
         -c_sf,                   -c_sf*a,                 (c_sf + c_tf),      0;
         -c_sr,                    c_sr*b,                  0,                (c_sr + c_tr) ];

    % 4. Undamped Modal Analysis: K * phi = omega_n^2 * M * phi
    [V_modes, D_lambda] = eig(K, M);
    omega_n_undamped = sqrt(diag(D_lambda));
    [omega_n_undamped, sort_idx] = sort(omega_n_undamped);
    V_modes = V_modes(:, sort_idx);
    fn_undamped = omega_n_undamped / (2 * pi);

    % 5. Damped State-Space Analysis: dx/dt = A_sys * x
    % State vector: x = [q; dq]
    A_sys = [ zeros(4), eye(4);
             -M \ K,   -M \ C ];

    eigenvalues = eig(A_sys);

    % Filter positive imaginary parts (damped conjugate pairs)
    pos_imag_idx = imag(eigenvalues) > 0;
    eig_pairs = eigenvalues(pos_imag_idx);

    wn = abs(eig_pairs);
    zeta = -real(eig_pairs) ./ wn;
    wd = imag(eig_pairs);
    fn = wn / (2 * pi);
    fd = wd / (2 * pi);

    % Sort by natural frequency
    [fn, sort_damped] = sort(fn);
    wn = wn(sort_damped);
    zeta = zeta(sort_damped);
    fd = fd(sort_damped);
    eig_sorted = eig_pairs(sort_damped);

    % Classify modes based on physical frequency ranges:
    % Modes < 3 Hz are sprung mass modes (bounce/heave and pitch)
    % Modes >= 3 Hz are unsprung mass wheel hop modes
    mode_names = cell(4, 1);
    mode_names{1} = 'Body Heave / Bounce';
    mode_names{2} = 'Body Pitch';
    
    % For unsprung modes, identify front vs rear from tyre deflection magnitude
    if abs(V_modes(3, 3)) >= abs(V_modes(4, 3))
        mode_names{3} = 'Front Wheel Hop';
        mode_names{4} = 'Rear Wheel Hop';
    else
        mode_names{3} = 'Rear Wheel Hop';
        mode_names{4} = 'Front Wheel Hop';
    end

    % Display Modal Summary Table
    fprintf('\n========================================================================\n');
    fprintf('           4-DOF HALF-CAR SUSPENSION: MODAL ANALYSIS VALIDATION         \n');
    fprintf('========================================================================\n');
    fprintf(' %-4s | %-22s | %-9s | %-9s | %-8s | %-12s\n', ...
            'Mode', 'Mode Description', 'fn (Hz)', 'fd (Hz)', 'Zeta (-)', 'Pole (s-plane)');
    fprintf('------------------------------------------------------------------------\n');
    for i = 1:4
        fprintf('  %d   | %-22s | %8.2f  | %8.2f  | %7.3f  | %6.2f +/- %5.2fj\n', ...
                i, mode_names{i}, fn(i), fd(i), zeta(i), real(eig_sorted(i)), imag(eig_sorted(i)));
    end
    fprintf('========================================================================\n\n');

    % Return structure
    modal.M = M;
    modal.K = K;
    modal.C = C;
    modal.A_sys = A_sys;
    modal.undamped_fn = fn_undamped;
    modal.mode_shapes = V_modes;
    modal.damped_fn = fn;
    modal.damped_fd = fd;
    modal.damping_ratio = zeta;
    modal.mode_names = mode_names;
    modal.poles = eig_sorted;
end
