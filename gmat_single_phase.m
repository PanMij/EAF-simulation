function G = gmat_single_phase(A, B, N1, N2, Nu)
%GMAT  Build the GPC dynamic matrix G from a CARIMA/ARIMA model.
%
%   G = gmat(A, B, N1, N2, Nu)
%
%   Model (noise ignored):
%       A(z^-1) * Δy(k) = z^-1 * B(z^-1) * Δu(k)
%
%   Inputs:
%     A  - row vector [1 a1 ... ana]  (denominator polynomial in z^-1)
%     B  - row vector [b0 b1 ... bnb] (numerator polynomial in z^-1)
%     N1 - minimum prediction step (>=1)
%     N2 - maximum prediction step (> N1)
%     Nu - control horizon (>=1, <= N2-N1+1 recommended)
%
%   Output:
%     G  - (N2-N1+1)-by-Nu dynamic matrix
%
%   Notes:
%     - Assumes one-sample input delay (z^-1) as in your equation:
%         ... = B(z^-1) Δu(k-1)
%       If your delay differs, shift B accordingly before calling.
%
%   Example:
%     A = [1 1.0584 -0.1774];
%     B = [-1.3157e-6 -1.8767e-6];
%     G = gmat(A, B, 1, 50, 10);
%
%   This implementation reuses the same SISO dynamic-matrix logic
%   as the internal helper in gpc_step.m.

    G = siso_gmat(A, B, N1, N2, Nu);
end

function G = siso_gmat(A, B, N1, N2, Nu)
    % ---- checks ----
    if nargin ~= 5
        error('gmat requires exactly 5 inputs: A, B, N1, N2, Nu.');
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
    validateattributes(Nu, {'numeric'}, {'scalar', 'integer', 'finite', '>=', 1}, ...
        mfilename, 'Nu');
    if N2 <= N1
        error('Require N2 > N1.');
    end

    na = numel(A) - 1;  % order of A
    nb = numel(B) - 1;  % order of B

    % ---- step 1: generate coefficients g(1..N2) by simulating unit Δu(k)=1 ----
    % We simulate the recursion:
    %   Δy(t) = -sum_{i=1..na} A(i+1)*Δy(t-i) + sum_{j=0..nb} B(j+1)*Δu(t-1-j)
    %   y(t)  = y(t-1) + Δy(t)
    %
    % with initial conditions y(0)=0, Δy(t)=0 for t<=0, and Δu(0)=1, else 0.
    %
    % Result: g(t) := y(t) for t=1..N2

    dy = zeros(1, N2 + na + 2);  % dy(idx) stores Δy at time (idx-1)
    y  = zeros(1, N2 + 2);       % y(idx)  stores y at time (idx-1)

    % helper: Δu(t) = 1 if t==0 else 0
    du_at = @(t) double(t == 0);

    for t = 1:N2
        acc = 0;
    
        % A-part: past Δy terms
        for i = 1:na
            if (t - i) + 1 >= 1
                acc = acc - A(i+1) * dy((t - i) + 1);
            end
        end
    
        % B-part: delayed Δu terms
        for j = 0:nb
            acc = acc + B(j+1) * du_at(t - 1 - j);
        end
    
        dy(t + 1) = acc;              % Δy(t)
        y(t + 1)  = y(t) + dy(t + 1); % y(t)
    end

    g = y(2:N2+1); % g(1)=y(1), ..., g(N2)=y(N2)

    % ---- step 2: fill G using Toeplitz shifts of g ----
    rows = N2 - N1 + 1;
    G = zeros(rows, Nu);

    for r = 1:rows
        for c = 1:Nu
            idx = N1 + r - c; % coefficient index into g
            if idx >= 1
                G(r, c) = g(idx);
            end
        end
    end
end
