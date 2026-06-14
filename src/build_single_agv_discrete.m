function model = build_single_agv_discrete(Ts, m)
%BUILD_SINGLE_AGV_DISCRETE Single-AGV continuous/discrete model.
%   model = build_single_agv_discrete(Ts, m)
%
% State x = [px; py; vx; vy], input u = [ax; ay].
% Continuous model:
%   x_dot = A*x + B*u
%
% Discrete model:
%   x(k+1) = Ad*x(k) + Bd*u(k)

    arguments
        Ts (1,1) double {mustBePositive}
        m  (1,1) double {mustBePositive}
    end

    A = [0 0 1 0;
         0 0 0 1;
         0 0 0 0;
         0 0 0 0];

    B = [0   0;
         0   0;
         1/m 0;
         0   1/m];

    % Exact ZOH discretization via block matrix exponential
    n = size(A, 1);
    nu = size(B, 2);
    M = [A, B;
         zeros(nu, n + nu)];
    Md = expm(M * Ts);

    Ad = Md(1:n, 1:n);
    Bd = Md(1:n, n+1:n+nu);

    model = struct();
    model.Ts = Ts;
    model.m = m;
    model.A = A;
    model.B = B;
    model.Ad = Ad;
    model.Bd = Bd;
    model.n = n;
    model.nu = nu;
end
