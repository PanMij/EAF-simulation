function G = gmat(A, B, N1, N2, Nu)
%GMAT  Build the MIMO GPC dynamic matrix G for a 3x3 CARIMA/ARIMA model.
%
%   G = gmat(A, B, N1, N2, Nu)
%
%   Model (noise ignored) for each path (i,j):
%       A{i,j}(z^-1) * Δy_i(k) = z^-1 * B{i,j}(z^-1) * Δu_j(k)
%
%   Inputs:
%     A  - 3x3x(na+1) numeric array with A(:,:,1) == 1
%     B  - 3x3x(nb+1) numeric array
%     N1 - minimum prediction step (>=1)
%     N2 - maximum prediction step (> N1)
%     Nu - control horizon (>=1, <= N2-N1+1 recommended)
%
%   Output:
%     G  - block dynamic matrix of size (3*(N2-N1+1)) x (3*Nu)
%
%   Notes:
%     - Assumes one-sample input delay (z^-1) for each path.
%     - If your delays differ per path, shift each B(:,:,k) before calling.
%
%   This implementation matches the internal gmat_mimo logic used by gpc_step.m.

    G = gmat_mimo(A, B, N1, N2, Nu);
end

function G = gmat_mimo(A, B, N1, N2, Nu)
    % ---- checks ----
    if nargin ~= 5
        error('gmat requires exactly 5 inputs: A, B, N1, N2, Nu.');
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
            Aij = squeeze(A(i, j, :)).';
            Bij = squeeze(B(i, j, :)).';
            Gij = gmat_single_phase(Aij, Bij, N1, N2, Nu);
            r0 = (i - 1) * rows + 1;
            c0 = (j - 1) * Nu + 1;
            G(r0:r0+rows-1, c0:c0+Nu-1) = Gij;
        end
    end
end
