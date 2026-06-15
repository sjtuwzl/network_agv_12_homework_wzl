function aug = build_augmented_packet_delay_model(Ad, Bd, d, gamma_mode)
%BUILD_AUGMENTED_PACKET_DELAY_MODEL Build delayed packet-loss augmented model.
%   aug = build_augmented_packet_delay_model(Ad, Bd, d, gamma_mode)
%
% Base plant:
%   x(k+1) = Ad*x(k) + Bd*u_a(k)
%
% Controller command:
%   u_c(k) = K*x(k)
%
% Networked actuation with fixed delay d and Bernoulli loss gamma(k):
%   u_a(k) = gamma(k)*u_c(k-d) + (1-gamma(k))*u_a(k-1)
%
% Augmented state (without closing K):
%   z(k) = [x(k);
%           x(k-1);
%           ...
%           x(k-d);
%           u_a(k-1)]
%
% Then:
%   z(k+1) = A0*z(k) + gamma(k)*B0*Y*z(k) + (1-gamma(k))*B1*z(k)
% where Y = K*C_pick and C_pick extracts x(k-d) from z.
%
% gamma_mode:
%   "loss-hold" => use hold-last-actuation (recommended)

    arguments
        Ad double
        Bd double
        d (1,1) double {mustBeInteger, mustBeNonnegative}
        gamma_mode (1,1) string = "loss-hold"
    end

    if gamma_mode ~= "loss-hold"
        error('Only gamma_mode = "loss-hold" is currently supported.');
    end

    n = size(Ad, 1);
    m = size(Bd, 2);
    if size(Ad, 2) ~= n
        error('Ad must be square.');
    end
    if size(Bd, 1) ~= n
        error('Bd row size must match Ad.');
    end

    nx_aug = (d + 1) * n + m;

    % z = [x0; x1; ...; xd; ua_prev], with xj = x(k-j)
    A0 = zeros(nx_aug);
    B1 = zeros(nx_aug);
    B0 = zeros(nx_aug, m);

    idx_x0 = 1:n;
    idx_ua = (d + 1) * n + (1:m);

    % x(k+1) = Ad*x(k) + Bd*u_a(k), and for gamma=0: u_a(k)=u_a(k-1)
    A0(idx_x0, idx_x0) = Ad;
    A0(idx_x0, idx_ua) = Bd;

    % Shift chain x(k-j+1) <- x(k-j), j=1..d
    for j = 1:d
        row = j*n + (1:n);
        col = (j-1)*n + (1:n);
        A0(row, col) = eye(n);
    end

    % Keep last actuation memory
    % ua(k) = gamma * u_c(k-d) + (1-gamma) * ua(k-1)
    % The (1-gamma) part goes to B1 * z
    B1(idx_ua, idx_ua) = eye(m);

    % gamma part maps to ua(k)
    B0(idx_ua, :) = eye(m);

    % Control picks x(k-d) from augmented state
    C_pick = zeros(n, nx_aug);
    idx_xd = d*n + (1:n);
    C_pick(:, idx_xd) = eye(n);

    aug = struct();
    aug.d = d;
    aug.n = n;
    aug.m = m;
    aug.nx_aug = nx_aug;
    aug.A0 = A0;
    aug.B0 = B0;
    aug.B1 = B1;
    aug.C_pick = C_pick;
    aug.gamma_mode = char(gamma_mode);
end
