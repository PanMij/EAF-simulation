function F = fmat(A, B, N1, N2, yk, dy_hist, du_hist)
%FMAT  Build the MIMO free-response vector F for a 3x3 CARIMA/ARIMA model.
%
%   F = fmat(A, B, N1, N2, yk, dy_hist, du_hist)
%
%   Model (noise ignored) for each input-output path (i,j):
%       A_ij(z^-1) * Δy_ij(k) = z^-1 * B_ij(z^-1) * Δu_j(k)
%   with total output y_i(k) = sum_j y_ij(k).
%
%   Inputs:
%     A       - 3x3x(na+1) numeric array with A(:,:,1) == 1
%     B       - 3x3x(nb+1) numeric array
%     N1      - minimum prediction step (>=1)
%     N2      - maximum prediction step (> N1)
%     yk      - 3x1 current output vector y(k)
%     dy_hist - 3xna matrix with rows:
%               [Δy_i(k) Δy_i(k-1) ...] (most recent first)
%     du_hist - 3x(nb+1) matrix with rows:
%               [Δu_j(k) Δu_j(k-1) ...] (most recent first)
%
%   Output:
%     F       - (3*(N2-N1+1))-by-1 free-response vector over steps N1..N2
%
%   Notes:
%     - Assumes one-sample input delay (z^-1). Future Δu(k+i)=0 for i>=1.
%
%   This implementation matches the internal fmat_mimo logic used by gpc_step.m.

    F = fmat_mimo(A, B, N1, N2, yk, dy_hist, du_hist);
end

function F = fmat_mimo(A, B, N1, N2, yk, dy_hist, du_hist)
    % ---- checks ----
    if nargin ~= 7
        error('fmat requires exactly 7 inputs: A, B, N1, N2, yk, dy_hist, du_hist.');
    end
    if ~isnumeric(A) || ~isnumeric(B) || ndims(A) ~= 3 || ndims(B) ~= 3
        error('A and B must be 3-D numeric arrays sized 3x3x(na+1) and 3x3x(nb+1).');
    end
    if size(A, 1) ~= 3 || size(A, 2) ~= 3 || size(B, 1) ~= 3 || size(B, 2) ~= 3
        error('A and B must be sized 3x3x(na+1) and 3x3x(nb+1).');
    end
    if any(A(:, :, 1) ~= 1, 'all')
        error('A(:,:,1) must be all ones (leading coefficient).');
    end
    validateattributes(N1, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'N1');
    validateattributes(N2, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'N2');
    if N2 <= N1
        error('Require N2 > N1.');
    end
    validateattributes(yk, {'numeric'}, {'vector', 'numel', 3, 'finite'}, ...
        mfilename, 'yk');
    yk = yk(:);

    % Model orders are inferred from the third dimension of A and B.
    na = size(A, 3) - 1;
    nb = size(B, 3) - 1;
    if ~isnumeric(dy_hist) || size(dy_hist, 1) ~= 3
        error('dy_hist must be a 3xna numeric matrix.');
    end
    if ~isnumeric(du_hist) || size(du_hist, 1) ~= 3
        error('du_hist must be a 3x(nb+1) numeric matrix.');
    end
    if na > 0 && size(dy_hist, 2) < na
        error('dy_hist must have at least %d columns.', na);
    end
    if size(du_hist, 2) < (nb + 1)
        error('du_hist must have at least %d columns.', nb + 1);
    end
    % Trim histories to the required lengths (most recent first).
    if na > 0
        dy_hist = dy_hist(:, 1:na);
    else
        dy_hist = zeros(3, 0);
    end
    du_hist = du_hist(:, 1:nb+1);

    rows = N2 - N1 + 1;
    F = zeros(3 * rows, 1);

    % Simulate free response for each output i with future Δu = 0.
    for i = 1:3
        dy_sim = zeros(3, N2);
        y_sim  = zeros(1, N2);
        y_prev = yk(i);

        for t = 1:N2
            % Accumulate Δy contributions from all inputs j.
            dy_sum = 0;
            for j = 1:3
                % Extract SISO coefficients for path (i,j).
                Aij = squeeze(A(i, j, :)).';
                Bij = squeeze(B(i, j, :)).';
                na_ij = numel(Aij) - 1;
                nb_ij = numel(Bij) - 1;

                % A-part: past Δy terms for the (i,j) path.
                acc = 0;
                for a = 1:na_ij
                    m = t - a;
                    if m <= 0
                        acc = acc - Aij(a+1) * dy_hist(i, -m + 1);
                    else
                        acc = acc - Aij(a+1) * dy_sim(j, m);
                    end
                end

                % B-part: delayed Δu terms for the (i,j) path.
                for b = 0:nb_ij
                    n = t - 1 - b;
                    if n <= 0
                        acc = acc + Bij(b+1) * du_hist(j, -n + 1);
                    end
                end

                % Store per-input Δy_ij and sum for output i.
                dy_sim(j, t) = acc;
                dy_sum = dy_sum + acc;
            end

            % Integrate Δy to get y, then store the predicted output.
            y_prev   = y_prev + dy_sum;
            y_sim(t) = y_prev;
        end

        % Pack output i predictions into the stacked F vector.
        Fi = y_sim(N1:N2).';
        r0 = (i - 1) * rows + 1;
        F(r0:r0+rows-1) = Fi;
    end
end
