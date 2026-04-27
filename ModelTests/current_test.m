clc;clear;
close all;

load("data/arc_data.mat");

% Downsample the data
Tsim = 1e-5; % Original sampling period
Fs = 5e3; % Downsampling frequency
N = floor(length(i_arc) * (Fs * Tsim)); % Number of samples after downsampling
dsIdx = floor(linspace(1, length(i_arc), N));
i_arc_ds = i_arc(dsIdx);
t_arc_ds = t_arc(dsIdx);

figure;
plot(t_arc_ds, i_arc_ds);


%% Perform FFT and plot the spectrum
res = fft(i_arc_ds);
res = fftshift(res);

L = length(i_arc_ds);
freqs = Fs/L*(-L/2:L/2-1);
figure;
plot(freqs, abs(res),"LineWidth",3);


%% Restore the original signal
NyqIdx = floor(L/2) + 1;
ssRes = res(NyqIdx:end);
freqs = freqs(NyqIdx:end);
[pks, locs] = findpeaks(abs(ssRes));
pks = 2 / L * pks;
angs = angle(ssRes);

x = linspace(0, 0.2, 20000);
y = zeros(size(x));
for i = 1:length(pks)
    y = y + pks(i) * cos(2*pi*freqs(locs(i))*x + angs(locs(i)));
end
figure;
plot(x, y);


%% Add harmonics to the signal
f0 = 50; % Fundamental frequency
N = 20; % Number of harmonics to add
A1 = max(pks); % Amplitude of the fundamental frequency
bRate = 0.5;
alpha = 0.5 + 0.5 * bRate; % Decay factor for the harmonics
c = 1e4 * (1 - bRate);
hm_ls = zeros(N, 1);
for i = 1 : N
    if mod(i, 2) == 0
        hm_ls(i) = A1 * exp(-alpha * (i - 1)) + c;
    else
        hm_ls(i) = c;
    end
end
figure;
plot(hm_ls, 'o-');

res = fftshift(res);
Ahar = zeros(N, 1);
for i = 1:N
    k = round(i * f0 / Fs * L);
    dA = L/2 * hm_ls(i) * exp(1i * angle(res(1+k)));
    res(1+k) = res(1+k) + dA;
    res(end-k+1) = res(end-k+1) + conj(dA);
    Ahar(i) = abs(res(k+1));
end
figure;
plot((0:L-1)*Fs/L, abs(res),"LineWidth",3);

figure;
plot(t_arc_ds, ifft(res));