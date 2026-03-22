function F = fmat(A, B, C, N1, N2, yk, dy_hist, du_hist, e_hist)
%FMAT  Build the MIMO free-response vector F for a 3x3 CARIMA model.
%
%   F = fmat(A, B, C, N1, N2, yk, dy_hist, du_hist, e_hist)
%
%   Model (incremental CARIMA):
%       A(z^-1) * Δy(k) = z^-1 * B(z^-1) * Δu(k) + C(z^-1) * e(k)
%
%   Inputs:
%     A       - 3x3x(na+1), A(:,:,1) = I (or ones in your convention)
%     B       - 3x3x(nb+1)
%     C       - 3x3x(nc+1), C(:,:,1) = I
%     N1,N2   - prediction range
%     yk      - 3x1 current output y(k)
%     dy_hist - 3xna   [Δy(k) Δy(k-1) ...] (most recent first)
%     du_hist - 3x(nb+1) [Δu(k) Δu(k-1) ...] (most recent first)
%     e_hist  - 3xnc   [e(k) e(k-1) ...] (most recent first)
%
%   Output:
%     F       - (3*(N2-N1+1)) x 1 stacked free-response vector
%
%   Notes:
%     - Future innovations e(k+1), e(k+2), ... are assumed zero
%       (certainty-equivalence predictor), so noise affects F only through e_hist.

    F = fmat_mimo_carima(A, B, C, N1, N2, yk, dy_hist, du_hist, e_hist);
end

function F = fmat_mimo_carima(A, B, C, N1, N2, yk, dy_hist, du_hist, e_hist)
    % ---- checks ----
    assert(nargin == 9);

    assert(isnumeric(A) && isnumeric(B) && isnumeric(C) && ...
           ndims(A) == 3 && ndims(B) == 3);

    assert(size(A,1) == 3 && size(A,2) == 3 && ...
           size(B,1) == 3 && size(B,2) == 3 && ...
           size(C,1) == 3 && size(C,2) == 3);

    assert(isscalar(N1) && isnumeric(N1) && isfinite(N1) && ...
           N1 >= 1 && N1 == floor(N1));

    assert(isscalar(N2) && isnumeric(N2) && isfinite(N2) && ...
           N2 > N1 && N2 == floor(N2));

    assert(isnumeric(yk) && numel(yk) == 3 && all(isfinite(yk(:))));

    yk = yk(:);

    na = size(A,3) - 1;
    nb = size(B,3) - 1;
    nc = size(C,3) - 1;

    assert(size(dy_hist,1) == 3 && size(dy_hist,2) >= na);
    assert(size(du_hist,1) == 3 && size(du_hist,2) >= (nb+1));
    if nc > 0
        assert(size(e_hist,1) == 3 && size(e_hist,2) >= nc);
    end

    % Trim to required lengths
    if na > 0, dy_hist = dy_hist(:,1:na); else, dy_hist = zeros(3,0); end
    du_hist = du_hist(:,1:nb+1);
    if nc > 0, e_hist = e_hist(:,1:nc); else, e_hist = zeros(3,0); end

    rows = N2 - N1 + 1;
    F = zeros(3*rows, 1);

    % Predicted incremental outputs Δy(k+t|k), t=1..N2
    dy_pred = zeros(3, N2);
    y_pred  = zeros(3, N2);

    y_prev = yk;  % y(k)

    for t = 1:N2
        dy_t = zeros(3,1);

        for i = 1:3
            acc = 0;

            % ---- A-part: -A1*Δy(k+t-1) - ... ----
            for a = 1:na
                idx = t - a;   % corresponds to Δy(k+idx)
                if idx <= 0
                    % known history: idx=0 -> Δy(k), idx=-1 -> Δy(k-1), ...
                    dy_vec = dy_hist(:, -idx + 1);
                else
                    % previously predicted
                    dy_vec = dy_pred(:, idx);
                end
                acc = acc - squeeze(A(i,:,a+1)) * dy_vec;
            end

            % ---- B-part: z^-1 B(z^-1) Δu(k+t) ----
            % With one-sample delay: term uses Δu(k+t-1-b)
            % Future Δu(k+1), Δu(k+2), ... are zero in free response.
            for b = 0:nb
                idx = t - 1 - b;   % corresponds to Δu(k+idx)
                if idx <= 0
                    du_vec = du_hist(:, -idx + 1);
                else
                    du_vec = zeros(3,1);  % future free response => zero
                end
                acc = acc + squeeze(B(i,:,b+1)) * du_vec;
            end

            % ---- C-part: C1*e(k+t-1) + ... + Cnc*e(k+t-nc) ----
            % Future innovations e(k+1), e(k+2), ... assumed zero.
            for c = 1:nc
                idx = t - c;   % corresponds to e(k+idx)
                if idx <= 0
                    e_vec = e_hist(:, -idx + 1);
                else
                    e_vec = zeros(3,1);   % future innovation = 0
                end
                acc = acc + squeeze(C(i,:,c+1)) * e_vec;
            end

            dy_t(i) = acc;
        end

        dy_pred(:,t) = dy_t;
        y_prev = y_prev + dy_t;
        y_pred(:,t) = y_prev;
    end

    for i = 1:3
        r0 = (i-1)*rows + 1;
        F(r0:r0+rows-1) = y_pred(i, N1:N2).';
    end
end