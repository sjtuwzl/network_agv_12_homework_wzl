function met = compute_performance_metrics(sim, Ts)
%COMPUTE_PERFORMANCE_METRICS Basic performance metrics for report plots.

    e = sim.e;
    u = sim.u_act;

    pos_err = sqrt(e(1, :).^2 + e(2, :).^2);
    vel_err = sqrt(e(3, :).^2 + e(4, :).^2);
    u_norm = sqrt(sum(u.^2, 1));

    met = struct();
    met.rmse_pos = sqrt(mean(pos_err.^2));
    met.rmse_vel = sqrt(mean(vel_err.^2));
    met.max_pos_err = max(pos_err);
    met.max_vel_err = max(vel_err);
    met.control_rms = sqrt(mean(u_norm.^2));
    met.loss_rate_empirical = 1 - mean(sim.gamma);
    met.initial_pos_err = pos_err(1);
    met.final_pos_err = pos_err(end);
    met.decay_ratio = pos_err(end) / max(pos_err(1), eps);

    thr = 0.02; % 2cm position error threshold (assuming meter unit)
    idx = find(pos_err <= thr, 1, 'first');
    if isempty(idx)
        met.settle_time = inf;
    else
        met.settle_time = (idx - 1) * Ts;
    end

    % Relative settling: first time below 50% of initial position error.
    thr_rel = 0.5 * pos_err(1);
    idx_rel = find(pos_err <= thr_rel, 1, 'first');
    if isempty(idx_rel)
        met.settle_time_50pct = inf;
    else
        met.settle_time_50pct = (idx_rel - 1) * Ts;
    end
end
