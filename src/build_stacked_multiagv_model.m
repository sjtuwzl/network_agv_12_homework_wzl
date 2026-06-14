function stacked = build_stacked_multiagv_model(single, N)
%BUILD_STACKED_MULTIAGV_MODEL Build N-AGV block diagonal stacked model.
%   stacked = build_stacked_multiagv_model(single, N)
%
% single: struct returned by build_single_agv_discrete
% N: number of AGVs (e.g., 12)
%
% x_all = [x1; x2; ...; xN], u_all = [u1; u2; ...; uN]
% x_all(k+1) = A_all*x_all(k) + B_all*u_all(k)

    arguments
        single struct
        N (1,1) double {mustBeInteger, mustBePositive}
    end

    A_all = kron(eye(N), single.Ad);
    B_all = kron(eye(N), single.Bd);

    stacked = struct();
    stacked.N = N;
    stacked.Ad = A_all;
    stacked.Bd = B_all;
    stacked.n_single = single.n;
    stacked.nu_single = single.nu;
    stacked.n = size(A_all, 1);
    stacked.nu = size(B_all, 2);
end
