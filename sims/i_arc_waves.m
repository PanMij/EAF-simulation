%% Plot arc current waveform and spectrum under poor/good buried-arc states
clear; clc; close all;

% ===== User settings =====
sigName   = 'i_arc';   % logged signal name in out.logsout
phaseIdx  = 1;         % 1: A phase, 2: B phase, 3: C phase
f0        = 50;        % fundamental frequency (Hz)
T_state   = 5;         % first 5 s: poor, last 5 s: good
nCyclePlot = 4;        % plot 4 cycles
fMaxPlot  = 1000;      % max frequency shown in spectrum (Hz)

% ===== Read logged signal =====
load("data/i_arc_log.mat");

if ~exist('out', 'var')
    error('变量 out 不存在，请先运行仿真并确保 out 在工作区中。');
end

if isa(out, 'Simulink.SimulationOutput')
    logs = out.logsout;
else
    error('变量 out 不是 Simulink.SimulationOutput 类型。');
end

sig = logs.getElement(sigName);
if isempty(sig)
    error('在 out.logsout 中未找到名为 "%s" 的 logged signal。', sigName);
end

vals = sig.Values;
t = vals.Time;
x = vals.Data;

% reshape to [Nt x Ny]
if isvector(x)
    x = x(:);
else
    sz = size(x);
    Nt = numel(t);
    if sz(1) == Nt
        x = reshape(x, Nt, []);
    else
        order = [ndims(x), 1:ndims(x)-1];
        x = reshape(permute(x, order), Nt, []);
    end
end

if phaseIdx > size(x, 2)
    error('phaseIdx=%d 超出信号列数范围，当前信号只有 %d 列。', phaseIdx, size(x, 2));
end

i_arc = x(:, phaseIdx);
dt = median(diff(t));
Ts = dt;
fs = 1 / Ts;

% ===== Time windows =====
T4 = nCyclePlot / f0;

% poor state waveform: take 4 cycles before 5 s
tPoorEnd   = T_state - Ts;
tPoorStart = tPoorEnd - T4;

% good state waveform: take 4 cycles after 5 s
tGoodStart = T_state + Ts;
tGoodEnd   = tGoodStart + T4;

idxPoorWave = (t >= tPoorStart) & (t <= tPoorEnd);
idxGoodWave = (t >= tGoodStart) & (t <= tGoodEnd);

idxPoorSpec = (t >= 0) & (t < T_state);
idxGoodSpec = (t >= T_state) & (t <= 2*T_state);

if ~any(idxPoorWave) || ~any(idxGoodWave) || ~any(idxPoorSpec) || ~any(idxGoodSpec)
    error('时间窗口内没有足够数据，请检查仿真时长和 logged signal。');
end

tPoorWave = t(idxPoorWave);
xPoorWave = i_arc(idxPoorWave);

tGoodWave = t(idxGoodWave);
xGoodWave = i_arc(idxGoodWave);

xPoorSpec = i_arc(idxPoorSpec);
xGoodSpec = i_arc(idxGoodSpec);

% ===== Spectrum calculation =====
[fPoor, APoor] = calcSpectrum(xPoorSpec, fs);
[fGood, AGood] = calcSpectrum(xGoodSpec, fs);

idxFPoor = fPoor <= fMaxPlot;
idxFGood = fGood <= fMaxPlot;

% ===== Plot =====
phaseName = {'A相', 'B相', 'C相'};
if phaseIdx <= 3
    phaseText = phaseName{phaseIdx};
else
    phaseText = sprintf('第%d相', phaseIdx);
end

fig = figure( ...
    'Name', ['埋弧状态电流波形与频谱 - ', phaseText], ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% 左上：poor waveform
ax1 = nexttile;
plot(tPoorWave, xPoorWave, 'LineWidth', 1.5, 'Color', 'b');
grid(ax1, 'on');
ax1.Color = 'w';
ax1.XColor = 'k';
ax1.YColor = 'k';
ax1.GridColor = [0.75 0.75 0.75];
ax1.GridAlpha = 0.35;
ax1.FontSize = 12;
ax1.LineWidth = 1;
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('电流 (A)', 'Color', 'k', 'FontSize', 12);
title([phaseText, ' 埋弧状态较差时的电流波形'], 'Color', 'k', 'FontSize', 13);

% 右上：poor spectrum
ax2 = nexttile;
plot(fPoor(idxFPoor), APoor(idxFPoor), 'LineWidth', 1.5, 'Color', 'r');
grid(ax2, 'on');
ax2.Color = 'w';
ax2.XColor = 'k';
ax2.YColor = 'k';
ax2.GridColor = [0.75 0.75 0.75];
ax2.GridAlpha = 0.35;
ax2.FontSize = 12;
ax2.LineWidth = 1;
xlabel('频率 (Hz)', 'Color', 'k', 'FontSize', 12);
ylabel('幅值', 'Color', 'k', 'FontSize', 12);
title([phaseText, ' 埋弧状态较差时的频谱图'], 'Color', 'k', 'FontSize', 13);
xlim(ax2, [0 fMaxPlot]);

% 左下：good waveform
ax3 = nexttile;
plot(tGoodWave, xGoodWave, 'LineWidth', 1.5, 'Color', 'b');
grid(ax3, 'on');
ax3.Color = 'w';
ax3.XColor = 'k';
ax3.YColor = 'k';
ax3.GridColor = [0.75 0.75 0.75];
ax3.GridAlpha = 0.35;
ax3.FontSize = 12;
ax3.LineWidth = 1;
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('电流 (A)', 'Color', 'k', 'FontSize', 12);
title([phaseText, ' 埋弧状态较好时的电流波形'], 'Color', 'k', 'FontSize', 13);

% 右下：good spectrum
ax4 = nexttile;
plot(fGood(idxFGood), AGood(idxFGood), 'LineWidth', 1.5, 'Color', 'r');
grid(ax4, 'on');
ax4.Color = 'w';
ax4.XColor = 'k';
ax4.YColor = 'k';
ax4.GridColor = [0.75 0.75 0.75];
ax4.GridAlpha = 0.35;
ax4.FontSize = 12;
ax4.LineWidth = 1;
xlabel('频率 (Hz)', 'Color', 'k', 'FontSize', 12);
ylabel('幅值', 'Color', 'k', 'FontSize', 12);
title([phaseText, ' 埋弧状态较好时的频谱图'], 'Color', 'k', 'FontSize', 13);
xlim(ax4, [0 fMaxPlot]);

sgtitle([phaseText, ' 电弧电流波形与频谱对比'], ...
    'Color', 'k', 'FontSize', 14);

exportgraphics(fig, 'i_arc_buried_arc_waveform_spectrum.png', 'Resolution', 300);

%% Local function
function [f, A] = calcSpectrum(x, fs)
    x = x(:);
    x = x - mean(x);          % remove DC component

    N = length(x);
    X = fft(x) / N;

    if rem(N, 2) == 0
        % even length
        A = abs(X(1:N/2+1));
        A(2:end-1) = 2 * A(2:end-1);
        f = (0:N/2)' * fs / N;
    else
        % odd length
        A = abs(X(1:(N+1)/2));
        A(2:end) = 2 * A(2:end);
        f = (0:(N-1)/2)' * fs / N;
    end
end