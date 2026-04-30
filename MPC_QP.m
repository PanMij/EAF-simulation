function du = MPC_QP(y, r, A, B, N1, N2, Nu, alpha, lam, reset, ...
                    C, D, wy, umin, umax, dumin, dumax, u_init, du_init, ...
                    Qkf, use_quadprog, is_active, u_applied, solve_when_inactive, Rkf, P0, ...
                    use_steady_state_kf, L)
%MPC_QP Constrained MPC wrapper with optional supervisor inputs.
%   This wrapper preserves the public MATLAB entry point and delegates the
%   implementation to MPC_QP_core so Simulink and script callers can share
%   the same controller logic.

    if nargin < 22
        is_active = [];
    end
    if nargin < 23
        u_applied = [];
    end
    if nargin < 24
        solve_when_inactive = [];
    end
    if nargin < 25
        Rkf = [];
    end
    if nargin < 26
        P0 = [];
    end
    if nargin < 27
        use_steady_state_kf = false;
    end
    if nargin < 28
        L = [];
    end

    du = MPC_QP_core(y, r, A, B, N1, N2, Nu, alpha, lam, reset, ...
                     C, D, wy, umin, umax, dumin, dumax, u_init, du_init, ...
                     Qkf, Rkf, P0, use_quadprog, is_active, u_applied, ...
                     solve_when_inactive, use_steady_state_kf, L);
end
