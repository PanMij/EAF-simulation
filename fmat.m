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
%     A       - 3x3 cell array of denominator polynomials (row vectors)
%     B       - 3x3 cell array of numerator polynomials (row vectors)
%     N1      - minimum prediction step (>=1)
%     N2      - maximum prediction step (> N1)
%     yk      - 3x1 current output vector y(k)
%     dy_hist - 3x3 cell array with entries:
%               [Δy_ij(k) Δy_ij(k-1) ...] (most recent first)
%               If a 3x1 cell array or 3xM matrix is provided, it is reused
%               for all j for the corresponding output i.
%     du_hist - 3x1 cell array or 3xM matrix with rows:
%               [Δu_j(k) Δu_j(k-1) ...] (most recent first)
%
%   Output:
%     F       - (3*(N2-N1+1))-by-1 free-response vector over steps N1..N2
%
%   Notes:
%     - Assumes one-sample input delay (z^-1). Future Δu(k+i)=0 for i>=1.

    % ---- checks ----
    if nargin ~= 7
        error('fmat requires exactly 7 inputs: A, B, N1, N2, yk, dy_hist, du_hist.');
    end
    if ~iscell(A) || ~iscell(B) || ~isequal(size(A), [3 3]) || ~isequal(size(B), [3 3])
        error('A and B must be 3x3 cell arrays of row vectors.');
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

    % Normalize histories.
    if ~iscell(dy_hist)
        if ~isnumeric(dy_hist) || size(dy_hist, 1) ~= 3
            error('dy_hist must be a 3x3 cell array, 3x1 cell array, or 3xM numeric matrix.');
        end
        dy_hist = {dy_hist(1, :), dy_hist(2, :), dy_hist(3, :)};
    end
    if ~iscell(du_hist)
        if ~isnumeric(du_hist) || size(du_hist, 1) ~= 3
            error('du_hist must be a 3x1 cell array or 3xM numeric matrix.');
        end
        du_hist = {du_hist(1, :), du_hist(2, :), du_hist(3, :)};
    end
    if ~(isequal(size(dy_hist), [3 3]) || numel(dy_hist) == 3)
        error('dy_hist must be a 3x3 cell array or a 3-element cell array.');
    end
    if numel(du_hist) ~= 3
        error('du_hist must have 3 elements.');
    end

    % If dy_hist is 3x1, reuse it for all inputs j.
    if numel(dy_hist) == 3 && ~isequal(size(dy_hist), [3 3])
        dy_hist = {dy_hist{1}, dy_hist{1}, dy_hist{1}; ...
                   dy_hist{2}, dy_hist{2}, dy_hist{2}; ...
                   dy_hist{3}, dy_hist{3}, dy_hist{3}};
    end

    rows = N2 - N1 + 1;
    F = zeros(3 * rows, 1);

    for i = 1:3
        % Preprocess per-input histories and coefficients for output i.
        dy_i = cell(1, 3);
        du_i = cell(1, 3);
        Aij_all = cell(1, 3);
        Bij_all = cell(1, 3);
        na_all = zeros(1, 3);
        nb_all = zeros(1, 3);

        for j = 1:3
            Aij = A{i, j};
            Bij = B{i, j};
            if ~isvector(Aij) || ~isvector(Bij)
                error('Each A{i,j} and B{i,j} must be row/column vectors.');
            end
            Aij = Aij(:).';
            Bij = Bij(:).';
            if isempty(Aij) || Aij(1) ~= 1
                error('Each A{i,j} must be non-empty with A(1) == 1.');
            end

            na = numel(Aij) - 1;
            nb = numel(Bij) - 1;
            na_all(j) = na;
            nb_all(j) = nb;
            Aij_all{j} = Aij;
            Bij_all{j} = Bij;

            dyj = dy_hist{i, j};
            dyj = dyj(:).';
            if na > 0 && numel(dyj) < na
                error('dy_hist{%d,%d} must have at least %d elements.', i, j, na);
            end
            if na > 0
                dy_i{j} = dyj(1:na);
            else
                dy_i{j} = [];
            end

            duj = du_hist{j};
            duj = duj(:).';
            if numel(duj) < (nb + 1)
                error('du_hist{%d} must have at least %d elements.', j, nb + 1);
            end
            du_i{j} = duj(1:nb+1);
        end

        % ---- simulate free response for output i ----
        dy_sim = zeros(3, N2);  % per-input Δy_ij
        y_sim  = zeros(1, N2);
        y_prev = yk(i);

        for t = 1:N2
            dy_sum = 0;
            for j = 1:3
                Aij = Aij_all{j};
                Bij = Bij_all{j};
                na = na_all(j);
                nb = nb_all(j);

                acc = 0;
                for a = 1:na
                    m = t - a;
                    if m <= 0
                        acc = acc - Aij(a+1) * dy_i{j}(-m + 1);
                    else
                        acc = acc - Aij(a+1) * dy_sim(j, m);
                    end
                end

                for b = 0:nb
                    n = t - 1 - b;
                    if n <= 0
                        acc = acc + Bij(b+1) * du_i{j}(-n + 1);
                    end
                end

                dy_sim(j, t) = acc;
                dy_sum = dy_sum + acc;
            end

            y_prev   = y_prev + dy_sum;
            y_sim(t) = y_prev;
        end

        Fi = y_sim(N1:N2).';
        r0 = (i - 1) * rows + 1;
        F(r0:r0+rows-1) = Fi;
    end
end
