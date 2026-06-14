function out = verify_mss_fixedK_yalmip(aug, p, K_delay, opts)
%VERIFY_MSS_FIXEDK_YALMIP Verify MSS for fixed delayed-state gain K_delay.
%
% Closed-loop:
%   A_loss = A0 + B1
%   A_succ = A0 + B0*(K_delay*C_pick)
%
% Verify:
%   p*A_loss'*P*A_loss + (1-p)*A_succ'*P*A_succ - (1-rho)*P <= -eps*I

    if nargin < 4
        opts = struct();
    end
    if ~isfield(opts, 'solver');  opts.solver = "sedumi"; end
    if ~isfield(opts, 'rho');     opts.rho = 0.0; end
    if ~isfield(opts, 'eps_pd');  opts.eps_pd = 1e-7; end
    if ~isfield(opts, 'verbose'); opts.verbose = false; end
    if ~isfield(opts, 'normalize_trace'); opts.normalize_trace = true; end

    nx = aug.nx_aug;
    A0 = aug.A0;
    B0 = aug.B0;
    B1 = aug.B1;
    C_pick = aug.C_pick;

    Kbar = K_delay * C_pick; % m x nx
    A_loss = A0 + B1;
    A_succ = A0 + B0 * Kbar;

    P = sdpvar(nx, nx, 'symmetric');
    L = p * (A_loss' * P * A_loss) + (1 - p) * (A_succ' * P * A_succ) - (1 - opts.rho) * P;

    cons = [P >= opts.eps_pd * eye(nx), L <= -opts.eps_pd * eye(nx)];
    if opts.normalize_trace
        cons = [cons, trace(P) == 1];
    end

    yopts = sdpsettings('solver', char(opts.solver), 'verbose', double(opts.verbose));
    diagnostics = optimize(cons, 0, yopts);

    out = struct();
    out.problem = diagnostics.problem;
    out.info = diagnostics.info;
    out.feasible = diagnostics.problem == 0;
    out.rho = opts.rho;
    out.solver = char(opts.solver);
    out.K_delay_state = K_delay;
    out.Kbar = Kbar;
    if out.feasible
        out.P = value(P);
    else
        out.P = [];
    end
end
