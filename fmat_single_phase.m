function F = fmat_single_phase(A, B, N1, N2, yk, dy_hist, du_hist)
%FMAT  Build the GPC free-response vector F for a CARIMA/ARIMA model.
%
%   F = fmat(A, B, N1, N2, yk, dy_hist, du_hist)
%
%   Model (noise ignored):
%       A(z^-1) * Δy(k) = z^-1 * B(z^-1) * Δu(k)
%
%   Inputs:
%     A        - row vector [1 a1 ... ana]  (denominator polynomial in z^-1)
%     B        - row vector [b0 b1 ... bnb] (numerator polynomial in z^-1)
%     N1       - minimum prediction step (>=1)
%     N2       - maximum prediction step (> N1)
%     yk       - current output y(k)
%     dy_hist  - [Δy(k) Δy(k-1) ... Δy(k-na+1)] (most recent first)
%     du_hist  - [Δu(k) Δu(k-1) ... Δu(k-nb)]  (most recent first)
%
%   Output:
%     F        - (N2-N1+1)-by-1 free-response vector over steps N1..N2
%
%   Notes:
%     - Assumes one-sample input delay (z^-1). Future Δu(k+i)=0 for i>=1.
%     - dy_hist must be at least length na, du_hist at least length nb+1.
%
%   Example:
%     A = [1 1.0584 -0.1774];
%     B = [-1.3157e-6 -1.8767e-6];
%     F = fmat(A, B, 1, 50, yk, [dyk dykm1], [duk dukm1]);

    % ---- checks ----
    if nargin ~= 7
        error('fmat requires exactly 7 inputs: A, B, N1, N2, yk, dy_hist, du_hist.');
    end
    if ~isvector(A) || ~isvector(B)
        error('A and B must be row/column vectors.');
    end
    A = A(:).';  % ensure row
    B = B(:).';  % ensure row
    if isempty(A) || A(1) ~= 1
        error('A must be non-empty with A(1) == 1.');
    end
    validateattributes(N1, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'N1');
    validateattributes(N2, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'N2');
    if N2 <= N1
        error('Require N2 > N1.');
    end
    validateattributes(yk, {'numeric'}, {'scalar', 'finite'}, mfilename, 'yk');

    na = numel(A) - 1;  % order of A
    nb = numel(B) - 1;  % order of B

    dy_hist = dy_hist(:).';
    du_hist = du_hist(:).';
    if na > 0 && numel(dy_hist) < na
        error('dy_hist must have at least %d elements.', na);
    end
    if numel(du_hist) < (nb + 1)
        error('du_hist must have at least %d elements.', nb + 1);
    end
    if na > 0
        dy_hist = dy_hist(1:na);
    else
        dy_hist = [];
    end
    du_hist = du_hist(1:nb+1);

    % ---- simulate free response with future Δu = 0 ----
    dy_sim = zeros(1, N2);  % Δy(k+1) ... Δy(k+N2)
    y_sim  = zeros(1, N2);  %  y(k+1) ...  y(k+N2)
    y_prev = yk;

    for t = 1:N2
        acc = 0;

        % A-part: past Δy terms
        for i = 1:na
            m = t - i; % index relative to k
            if m <= 0
                acc = acc - A(i+1) * dy_hist(-m + 1);
            else
                acc = acc - A(i+1) * dy_sim(m);
            end
        end

        % B-part: delayed Δu terms
        for j = 0:nb
            n = t - 1 - j; % index relative to k
            if n <= 0
                acc = acc + B(j+1) * du_hist(-n + 1);
            end
        end

        dy_sim(t) = acc;       % Δy(k+t)
        y_prev    = y_prev + acc;
        y_sim(t)  = y_prev;    % y(k+t)
    end

    F = y_sim(N1:N2).';
end
