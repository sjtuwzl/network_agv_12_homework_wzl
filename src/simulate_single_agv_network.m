function sim = simulate_single_agv_network(Ad, Bd, K, d, p, steps, x0, x_ref, seed, use_delay_state_feedback)
%SIMULATE_SINGLE_AGV_NETWORK Closed-loop simulation under packet loss + delay.
%
% Plant:
%   x(k+1) = Ad*x(k) + Bd*u_a(k)
%
% Networked actuation:
%   u_a(k) = gamma(k)*u_c(k-d) + (1-gamma(k))*u_a(k-1)
%   gamma(k)~Bernoulli(1-p), gamma=1 means success.
%
% Controller:
%   u_c(j) = K*(x(j)-x_ref)

    arguments
        Ad double
        Bd double
        K double
        d (1,1) double {mustBeInteger, mustBeNonnegative}
        p (1,1) double {mustBeGreaterThanOrEqual(p, 0), mustBeLessThan(p, 1)}
        steps (1,1) double {mustBeInteger, mustBePositive}
        x0 double
        x_ref double
        seed (1,1) double {mustBeInteger} = 1
        use_delay_state_feedback (1,1) logical = true
    end

    n = size(Ad, 1);
    m = size(Bd, 2);

    x = zeros(n, steps + 1);
    u_act = zeros(m, steps);
    u_cmd_delayed = zeros(m, steps);
    gamma = zeros(1, steps);

    x(:, 1) = x0(:);
    ua_prev = zeros(m, 1);

    rng(seed);

    for k = 1:steps
        if use_delay_state_feedback
            idx_feedback = k - d;
            if idx_feedback < 1
                x_fb = x(:, 1);
            else
                x_fb = x(:, idx_feedback);
            end
        else
            x_fb = x(:, k);
        end

        uc_delayed = K * (x_fb - x_ref(:));
        gk = rand() > p; % success=1 with prob 1-p

        ua = gk * uc_delayed + (1 - gk) * ua_prev;

        x(:, k+1) = Ad * x(:, k) + Bd * ua;

        u_cmd_delayed(:, k) = uc_delayed;
        u_act(:, k) = ua;
        gamma(k) = gk;
        ua_prev = ua;
    end

    e = x - x_ref(:);
    sim = struct();
    sim.x = x;
    sim.e = e;
    sim.u_act = u_act;
    sim.u_cmd_delayed = u_cmd_delayed;
    sim.gamma = gamma;
    sim.p = p;
    sim.d = d;
end
