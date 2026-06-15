function out = solve_mss_lmi_yalmip(aug, p, opts)
%SOLVE_MSS_LMI_YALMIP Solve MSS stabilizing gain via YALMIP.
%   out = solve_mss_lmi_yalmip(aug, p, opts)
%
% Model:
%   z(k+1) = [A0 + (1-gamma(k))B1 + gamma(k)B0*Kbar] z(k)
% with gamma(k) ~ Bernoulli(1-p), P(gamma=0)=p.
%
% MSS sufficient condition:
%   E{z(k+1)' P z(k+1)} - z(k)' P z(k) < 0
% enforced at two modes (gamma=0 and gamma=1):
%   p * A_loss' P A_loss + (1-p) * A_succ' P A_succ - P < 0
%
% Bilinear terms in K,P are convexified by Y = Kbar*Q with Q = P^{-1}
% using Schur complement:
%
% [ Q,  sqrt(p)*(A_loss*Q),  sqrt(1-p)*(A0*Q + B0*Y) ;
%   *,                Q,                           0 ;
%   *,                *,                           Q ] > 0
%
% where A_loss = A0 + B1, and Kbar = Y / Q.

    if nargin < 3
        opts = struct();
    end
    if ~isfield(opts, 'solver')
        opts.solver = "sdpt3";
    end
    if ~isfield(opts, 'eps_pd')
        opts.eps_pd = 1e-6;
    end
    if ~isfield(opts, 'verbose')
        opts.verbose = true;
    end
    if ~isfield(opts, 'rho')
        opts.rho = 0.0;
    end
    if ~isfield(opts, 'enforce_delay_structure')
        opts.enforce_delay_structure = true;
    end
    if ~isfield(opts, 'normalize_trace')
        opts.normalize_trace = true;
    end
    if ~isfield(opts, 'objective_mode')
        opts.objective_mode = "feasibility"; % "feasibility" or "min_trace"
    end

    validateattributes(p, {'double'}, {'scalar', '>=', 0, '<', 1}, mfilename, 'p');
    validateattributes(opts.rho, {'double'}, {'scalar', '>=', 0, '<', 1}, mfilename, 'opts.rho');

    if ~license('test', 'optimization_toolbox')
        % Not required by YALMIP itself, keep as non-fatal warning.
        warning('Optimization Toolbox not detected. YALMIP with SDP solver may still work.');
    end

    % Preflight checks: avoid calling non-YALMIP functions with same names.
    sdpvar_path = which('sdpvar');
    optimize_path = which('optimize');
    sdpsettings_path = which('sdpsettings');
    if isempty(sdpvar_path) || isempty(optimize_path) || isempty(sdpsettings_path)
        error(['YALMIP not found in path. Please add YALMIP first, e.g. ', ...
               'addpath(genpath(''path_to_yalmip''));']);
    end
    if ~contains(lower(optimize_path), 'yalmip')
        error(['Detected optimize.m is not from YALMIP: ', optimize_path, ...
               '. Put YALMIP earlier in path (addpath(...,''-begin'')).']);
    end

    nx = aug.nx_aug;
    m = aug.m;

    A0 = aug.A0;
    B0 = aug.B0;
    B1 = aug.B1;
    C_pick = aug.C_pick;
    A_loss = A0 + B1;

    % Decision variables
    Q = sdpvar(nx, nx, 'symmetric');
    Y = sdpvar(m, nx, 'full'); % Y = Kbar * Q

    AsQ = A0 * Q + B0 * Y;
    AlQ = A_loss * Q;

    % Enforce mean-square decay margin rho:
    % E[V(k+1)] <= (1-rho) V(k)
    M = [(1 - opts.rho) * Q,      sqrt(p) * AlQ,         sqrt(1 - p) * AsQ;
         (sqrt(p) * AlQ)',       Q,                     zeros(nx);
         (sqrt(1 - p) * AsQ)',   zeros(nx),             Q];

    cons = [];
    cons = [cons, Q >= opts.eps_pd * eye(nx)];
    cons = [cons, M >= opts.eps_pd * eye(3 * nx)];
    if opts.normalize_trace
        cons = [cons, trace(Q) == 1];
    end

    % Keep controller consistent with simulation: u uses x(k-d) only.
    % Since Y = Kbar*Q and Kbar should only act on delayed-state block,
    % enforce Y columns outside x(k-d) selection to be zero.
    if opts.enforce_delay_structure
        delay_cols = any(C_pick > 0, 1);
        cons = [cons, Y(:, ~delay_cols) == 0];
    end

    if strcmpi(char(opts.objective_mode), 'min_trace')
        objective = trace(Q);
    else
        objective = 0;
    end

    yopts = sdpsettings('solver', char(opts.solver), 'verbose', double(opts.verbose));
    diagnostics = optimize(cons, objective, yopts);

    out = struct();
    out.problem = diagnostics.problem;
    out.info = diagnostics.info;
    out.solver = char(opts.solver);
    out.rho = opts.rho;
    out.enforce_delay_structure = opts.enforce_delay_structure;
    out.normalize_trace = opts.normalize_trace;
    out.objective_mode = char(opts.objective_mode);
    out.path_check = struct('sdpvar', sdpvar_path, 'sdpsettings', sdpsettings_path, 'optimize', optimize_path);

    if diagnostics.problem ~= 0
        out.feasible = false;
        out.Kbar = [];
        out.K_delay_state = [];
        out.Q = [];
        out.P = [];
        return;
    end

    Qv = value(Q);
    Yv = value(Y);
    Kbar = Yv / Qv;

    % Effective gain on delayed plant state x(k-d)
    K_delay = Kbar * C_pick'; % size m x n

    out.feasible = true;
    out.Kbar = Kbar;
    out.K_delay_state = K_delay;
    out.Q = Qv;
    out.P = inv(Qv);
end
