function G = gmat(A, B, N1, N2, Nu)
%GMAT  Build the MIMO GPC dynamic matrix G for a 3x3 CARIMA/ARIMA model.
%
%   G = gmat(A, B, N1, N2, Nu)
%
%   Model (noise ignored) for each path (i,j):
%       A{i,j}(z^-1) * Δy_i(k) = z^-1 * B{i,j}(z^-1) * Δu_j(k)
%
%   Inputs:
%     A  - 3x3 cell array of denominator polynomials (row vectors)
%     B  - 3x3 cell array of numerator polynomials (row vectors)
%     N1 - minimum prediction step (>=1)
%     N2 - maximum prediction step (> N1)
%     Nu - control horizon (>=1, <= N2-N1+1 recommended)
%
%   Output:
%     G  - block dynamic matrix of size (3*(N2-N1+1)) x (3*Nu)
%
%   Notes:
%     - Assumes one-sample input delay (z^-1) for each path.
%     - If your delays differ per path, shift each B{i,j} before calling.

    % ---- checks ----
    if nargin ~= 5
        error('gmat requires exactly 5 inputs: A, B, N1, N2, Nu.');
    end
    if ~iscell(A) || ~iscell(B) || ~isequal(size(A), [3 3]) || ~isequal(size(B), [3 3])
        error('A and B must be 3x3 cell arrays of row vectors.');
    end
    validateattributes(N1, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'N1');
    validateattributes(N2, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'N2');
    validateattributes(Nu, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'Nu');
    if N2 <= N1
        error('Require N2 > N1.');
    end

    rows = N2 - N1 + 1;
    G = zeros(3 * rows, 3 * Nu);

    % Build each SISO block G_ij and place into the MIMO G matrix.
    for i = 1:3
        for j = 1:3
            Aij = A{i, j};
            Bij = B{i, j};
            Gij = siso_gmat(Aij, Bij, N1, N2, Nu);
            r0 = (i - 1) * rows + 1;
            c0 = (j - 1) * Nu + 1;
            G(r0:r0+rows-1, c0:c0+Nu-1) = Gij;
        end
    end
end

function G = siso_gmat(A, B, N1, N2, Nu)
    if ~isvector(A) || ~isvector(B)
        error('Each A{i,j} and B{i,j} must be row/column vectors.');
    end
    A = A(:).';  % ensure row
    B = B(:).';  % ensure row
    if isempty(A) || A(1) ~= 1
        error('Each A{i,j} must be non-empty with A(1) == 1.');
    end

    na = numel(A) - 1;
    nb = numel(B) - 1;

    dy = zeros(1, N2 + na + 2);
    y  = zeros(1, N2 + 2);
    du_at = @(t) double(t == 0);

    for t = 1:N2
        acc = 0;
        for i = 1:na
            if (t - i) + 1 >= 1
                acc = acc - A(i+1) * dy((t - i) + 1);
            end
        end
        for j = 0:nb
            acc = acc + B(j+1) * du_at(t - 1 - j);
        end
        dy(t + 1) = acc;
        y(t + 1)  = y(t) + dy(t + 1);
    end

    g = y(2:N2+1);

    rows = N2 - N1 + 1;
    G = zeros(rows, Nu);
    for r = 1:rows
        for c = 1:Nu
            idx = N1 + r - c;
            if idx >= 1
                G(r, c) = g(idx);
            end
        end
    end
end
