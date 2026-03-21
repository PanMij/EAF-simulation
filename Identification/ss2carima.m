%% ssinc_to_carima_demo.m
% Convert the incremental discrete-time state-space model
%
%   x(k+1)   = A x(k) + B du(k)
%   dy(k)    = C x(k)
%
% to the CARIMA form
%
%   a(q^-1) dy(k) = Bc(q^-1) du(k-1) + e(k)
%
% where C_noise(q^-1) = 1.
%
% Notes
% -----
% 1) This script assumes there is NO direct term from du(k) to dy(k),
%    i.e. D = 0.
% 2) The result is exact for the deterministic plant part.
% 3) a(q^-1) is a scalar polynomial (common denominator),
%    and Bc(q^-1) is a ny-by-nu polynomial matrix.
%
% Output structure:
%   carima.a         -> row vector [1 a1 a2 ... an]
%                       meaning a(q^-1)=1+a1 q^-1+...+an q^-n
%   carima.B         -> 3-D array, size (ny, nu, n)
%                       B(:,:,1) is coefficient of q^0
%                       B(:,:,2) is coefficient of q^-1
%                       ...
%                       so
%                       Bc(q^-1)=B(:,:,1)+B(:,:,2)q^-1+...+B(:,:,n)q^-(n-1)
%   carima.nx, ny, nu
%   carima.C_noise   -> 1
%
% Example CARIMA equation:
%   a(q^-1) * dy(k) = Bc(q^-1) * du(k-1) + e(k)

clear; clc;

%% ===== USER INPUT =====
% Replace these by your own identified matrices.
% Example:
% A = [...];
% B = [...];
% C = [...];

load("data/noise_40/MPC_params_pem_40.mat");

tol = 1e-10;

%% ===== CONVERT =====
carima = ssinc2carima(A, B, C, tol);

%% ===== DISPLAY RESULTS =====
fprintf('\n=== CARIMA model ===\n');
fprintf('a(q^-1) = %s\n\n', poly_qinv_to_str(carima.a, 'q^{-1}', tol));

fprintf('Bc(q^-1) coefficient matrices:\n');
fprintf('Bc(q^-1) = B0 + B1 q^-1 + ... + B_{n-1} q^{-(n-1)}\n\n');

for k = 1:size(carima.B, 3)
    fprintf('B%d = coefficient of q^{-%d}\n', k-1, k-1);
    disp(carima.B(:,:,k));
end

fprintf('\nIndividual channel polynomials B_ij(q^-1):\n');
for iy = 1:carima.ny
    for iu = 1:carima.nu
        bij = squeeze(carima.B(iy, iu, :)).';
        fprintf('B_{%d,%d}(q^-1) = %s\n', iy, iu, ...
            poly_qinv_to_str(bij, 'q^{-1}', tol));
    end
end

fprintf('\nEquivalent CARIMA form:\n');
fprintf('a(q^-1) * dy(k) = Bc(q^-1) * du(k-1) + e(k)\n');
fprintf('with C_noise(q^-1) = 1.\n');

%% ===== OPTIONAL: VERIFY BY RECONSTRUCTING FIRST n MARKOV TERMS =====
% This section checks consistency numerically.
verify_carima(A, B, C, carima, tol);
maxErr = check_tf_equivalence(A, B, C, carima);

N = 200;
nu = size(B,2);
ny = size(C,1);
nx = size(A,1);

% du = randn(N,nu);
load("data/noise_40/IdMPC_val.mat", "u", "y_real");
u = u(20:end, :);
y = y_real(20:end, :);
du = diff(u);
dy = diff(y);

% state-space simulation
x = zeros(nx,1);
dy_ss = zeros(N,ny);
for k = 1:N
    dy_ss(k,:) = (C*x).';
    x = A*x + B*du(k,:).';
end

% CARIMA simulation
a = carima.a;                  % [1 a1 ... an]
Bp = carima.B;                 % B0, B1, ...
na = numel(a)-1;
nb = size(Bp,3);

dy_ca = zeros(N,ny);

for k = 1:N
    yk = zeros(ny,1);

    % -a1*dy(k-1)-...-an*dy(k-n)
    for i = 1:min(na,k-1)
        yk = yk - a(i+1) * dy_ca(k-i,:).';
    end

    % B0*du(k-1)+B1*du(k-2)+...
    for j = 1:min(nb,k-1)
        yk = yk + Bp(:,:,j) * du(k-j,:).';
    end

    dy_ca(k,:) = yk.';
end

max_err = max(abs(dy_ss(:) - dy_ca(:)));
disp(max_err)

%% ===== SAVE THE RESULTS =====
A = repmat(eye(3), [1 1 numel(carima.a)]);
for k = 1 : size(A, 3)
    A(:, :, k) = A(:, :, k) * carima.a(k);
end
B = carima.B;
C = eye(3);


%% =======================================================================
function carima = ssinc2carima(A, B, C, tol)
%SSINC2CARIMA Convert incremental SS model to CARIMA with C(q^-1)=1
%
% Input:
%   A, B, C : state-space matrices in
%             x(k+1)=A x(k)+B du(k), dy(k)=C x(k)
%   tol     : numerical tolerance
%
% Output:
%   carima  : struct with fields a, B, nx, ny, nu, C_noise

    arguments
        A double
        B double
        C double
        tol double = 1e-10
    end

    [nx1, nx2] = size(A);
    [nxB, nu]  = size(B);
    [ny, nxC]  = size(C);

    if nx1 ~= nx2
        error('A must be square.');
    end
    if nxB ~= nx1
        error('B must have the same number of rows as A.');
    end
    if nxC ~= nx1
        error('C must have the same number of columns as A.');
    end

    nx = nx1;

    % Characteristic polynomial of A:
    % det(zI - A) = z^n + a1 z^(n-1) + ... + an
    %
    % Therefore, in q^-1 = z^-1:
    % a(q^-1) = 1 + a1 q^-1 + ... + an q^-n
    a = poly(A);
    a = cleanup_small(a, tol);

    % Markov parameters of the incremental model:
    % G(q^-1) = dy/du = H1 q^-1 + H2 q^-2 + ...
    % where Hk = C A^(k-1) B
    H = zeros(ny, nu, nx);
    Ak = eye(nx);
    for k = 1:nx
        H(:,:,k) = C * Ak * B;   % H_k = C A^(k-1) B
        Ak = A * Ak;
    end
    H = cleanup_small(H, tol);

    % Build numerator polynomial matrix:
    %
    % If
    %   G(q^-1) = q^-1 * Bc(q^-1) / a(q^-1),
    %
    % then
    %   a(q^-1) G(q^-1)
    % gives coefficients of q^-1, q^-2, ..., q^-n
    % and these are exactly B0, B1, ..., B_{n-1}.
    %
    % So:
    %   Bc(q^-1) = B0 + B1 q^-1 + ... + B_{n-1} q^-(n-1)
    %
    % with
    %   B_{l-1} = H_l + a1 H_{l-1} + ... + a_{l-1} H_1
    %
    % for l = 1,2,...,n.
    Bpoly = zeros(ny, nu, nx);

    % a = [1 a1 a2 ... an]
    for l = 1:nx
        M = H(:,:,l);
        for i = 1:l-1
            M = M + a(i+1) * H(:,:,l-i);
        end
        Bpoly(:,:,l) = cleanup_small(M, tol);
    end

    carima = struct();
    carima.a       = a;
    carima.B       = Bpoly;
    carima.nx      = nx;
    carima.ny      = ny;
    carima.nu      = nu;
    carima.C_noise = 1;
end


function s = poly_qinv_to_str(coeff, varname, tol)
%POLY_QINV_TO_STR Convert coefficients to a readable q^-1 polynomial string
%
% coeff = [c0 c1 c2 ... cm]
% means c0 + c1 q^-1 + c2 q^-2 + ... + cm q^-m

    coeff = coeff(:).';
    coeff(abs(coeff) < tol) = 0;

    terms = {};
    for k = 1:numel(coeff)
        c = coeff(k);
        p = k - 1;

        if c == 0
            continue;
        end

        if p == 0
            base = sprintf('%.12g', c);
        elseif p == 1
            if abs(c - 1) < tol
                base = varname;
            elseif abs(c + 1) < tol
                base = ['-' varname];
            else
                base = sprintf('%.12g%s', c, varname);
            end
        else
            if abs(c - 1) < tol
                base = sprintf('%s^-%d', varname(1:end-4), p); %#ok<SPRINTFN>
                % fallback below if the above formatting is not desired
                base = sprintf('%s^%d', varname, p); % displays q^{-1}^p
            elseif abs(c + 1) < tol
                base = sprintf('-%s^%d', varname, p);
            else
                base = sprintf('%.12g%s^%d', c, varname, p);
            end
        end

        terms{end+1} = base; %#ok<AGROW>
    end

    if isempty(terms)
        s = '0';
        return;
    end

    s = terms{1};
    for i = 2:numel(terms)
        t = terms{i};
        if startsWith(t, '-')
            s = [s ' - ' t(2:end)]; %#ok<AGROW>
        else
            s = [s ' + ' t]; %#ok<AGROW>
        end
    end
end


function X = cleanup_small(X, tol)
%CLEANUP_SMALL Remove tiny real/imaginary parts caused by numerical error
    X(abs(X) < tol) = 0;

    if ~isreal(X)
        Xi = imag(X);
        Xr = real(X);

        Xi(abs(Xi) < tol) = 0;
        Xr(abs(Xr) < tol) = 0;

        if all(Xi(:) == 0)
            X = Xr;
        else
            X = complex(Xr, Xi);
        end
    end
end


function verify_carima(A, B, C, carima, tol)
%VERIFY_CARIMA Simple numerical verification
%
% Checks the first nx Markov parameters:
%   G(q^-1) = q^-1 * Bc(q^-1) / a(q^-1)
%
% against
%   H_k = C A^(k-1) B

    nx = carima.nx;
    ny = carima.ny;
    nu = carima.nu;
    a  = carima.a;
    Bp = carima.B;

    % Original Markov parameters
    H_true = zeros(ny, nu, nx);
    Ak = eye(size(A));
    for k = 1:nx
        H_true(:,:,k) = C * Ak * B;
        Ak = A * Ak;
    end

    % Reconstructed Markov parameters from CARIMA coefficients
    H_rec = zeros(ny, nu, nx);
    for k = 1:nx
        % From:
        %   H_k + a1 H_{k-1} + ... + a_{k-1} H_1 = B_{k-1}
        rhs = Bp(:,:,k);
        for i = 1:k-1
            rhs = rhs - a(i+1) * H_rec(:,:,k-i);
        end
        H_rec(:,:,k) = rhs;
    end

    err = norm(H_true(:) - H_rec(:), inf);

    fprintf('\nVerification:\n');
    fprintf('max abs error in first %d Markov parameters = %.3e\n', nx, err);

    if err < 100*tol
        fprintf('Verification passed.\n');
    else
        fprintf('Warning: verification error is larger than expected.\n');
    end
end

function err = check_tf_equivalence(A,B,C,carima)
    zlist = [1.1, 1.2, 1.5, 2.0, 1.1*exp(1j*0.4), 1.3*exp(1j*1.0)];
    err = zeros(numel(zlist),1);

    for k = 1:numel(zlist)
        z = zlist(k);

        Gss = C * ((z*eye(size(A)) - A) \ B);

        a_val = polyval(carima.a, z);   % since carima.a came from poly(A)
        B_val = zeros(carima.ny, carima.nu);
        nB = size(carima.B,3);
        for i = 1:nB
            B_val = B_val + carima.B(:,:,i) * z^(-(i-1));
        end

        Gcar = z^(-1) * B_val / a_val;
        err(k) = norm(Gss - Gcar, 'fro');
    end
end