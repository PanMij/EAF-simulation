%% Plot key signals of variable-impedance control system
clear; clc; close all;

load("data/vi_log.mat");

if ~exist('out', 'var')
    error('工作区中不存在变量 out，请先运行仿真。');
end

logs = out.logsout;
if isempty(logs) || logs.numElements == 0
    error('out.logsout 为空，未找到 logged signals。');
end

%% Get logged signals
[bRate,  t_bRate]  = getLoggedData(logs, {'bRate'});
[hy_out, t_hy]     = getLoggedData(logs, {'hy_out'});
[l_slag, t_slag]   = getLoggedData(logs, {'l_slag'});
[FSI,    t_FSI]    = getLoggedData(logs, {'FSI'});
[SampEn, t_SampEn] = getLoggedData(logs, {'SampEn'});
[Zadjk,  t_Zadjk]  = getLoggedData(logs, {'Z_adj(k)'});
[Rreal,  t_Rreal]  = getLoggedData(logs, {'R_real'});

% Reshape to [Nt x Ny]
bRate  = reshapeSignal(bRate,  t_bRate);
hy_out = reshapeSignal(hy_out, t_hy);
l_slag = reshapeSignal(l_slag, t_slag);
FSI    = reshapeSignal(FSI,    t_FSI);
SampEn = reshapeSignal(SampEn, t_SampEn);
Zadjk  = reshapeSignal(Zadjk,  t_Zadjk);
Rreal  = reshapeSignal(Rreal,  t_Rreal);

%% Drop data from 0 s to 1 s
tMinPlot = 1.0;
maxPlotPts = 10000;

idx_bRate  = t_bRate  > tMinPlot;
idx_hy     = t_hy     > tMinPlot;
idx_slag   = t_slag   > tMinPlot;
idx_FSI    = t_FSI    > tMinPlot;
idx_SampEn = t_SampEn > tMinPlot;
idx_Zadjk  = t_Zadjk  > tMinPlot;
idx_Rreal  = t_Rreal  > tMinPlot;

t_bRate = t_bRate(idx_bRate);
bRate   = bRate(idx_bRate, :);

t_hy   = t_hy(idx_hy);
hy_out = hy_out(idx_hy, :);

t_slag = t_slag(idx_slag);
l_slag = l_slag(idx_slag, :);

t_FSI = t_FSI(idx_FSI);
FSI   = FSI(idx_FSI, :);

t_SampEn = t_SampEn(idx_SampEn);
SampEn   = SampEn(idx_SampEn, :);

t_Zadjk = t_Zadjk(idx_Zadjk);
Zadjk   = Zadjk(idx_Zadjk, :);

t_Rreal = t_Rreal(idx_Rreal);
Rreal   = Rreal(idx_Rreal, :);

%% Downsample data for plotting only
[t_bRate_plot,  bRate_plot]  = downsampleForPlot(t_bRate,  bRate,  maxPlotPts);
[t_hy_plot,     hy_out_plot] = downsampleForPlot(t_hy,     hy_out, maxPlotPts);
[t_slag_plot,   l_slag_plot] = downsampleForPlot(t_slag,   l_slag, maxPlotPts);
[t_FSI_plot,    FSI_plot]    = downsampleForPlot(t_FSI,    FSI,    maxPlotPts);
[t_SampEn_plot, SampEn_plot] = downsampleForPlot(t_SampEn, SampEn, maxPlotPts);
[t_Zadjk_plot,  Zadjk_plot]  = downsampleForPlot(t_Zadjk,  Zadjk,  maxPlotPts);
[t_Rreal_plot,  Rreal_plot]  = downsampleForPlot(t_Rreal,  Rreal,  maxPlotPts);

% Basic checks
nPhase = size(Rreal, 2);
if nPhase < 3
    error('信号列数不足，当前仅检测到 %d 列，相数应为 3。', nPhase);
end

phaseName = {'A相', 'B相', 'C相'};
phaseColor = [ ...
    0.0000    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250];

%% Plot
fig = figure( ...
    'Name', 'Variable-impedance control results', ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% ===== 左上：bRate =====
ax1 = nexttile;
hold(ax1, 'on');
for k = 1:3
    plot(t_bRate_plot, bRate_plot(:, k), 'LineWidth', 1.5, 'Color', phaseColor(k, :));
end
styleAxes(ax1);
xlim(ax1, [tMinPlot, inf]);
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('bRate', 'Color', 'k', 'FontSize', 12);
title('各相埋弧状态 bRate', 'Color', 'k', 'FontSize', 13);
legend(ax1, phaseName, ...
    'Location', 'eastoutside', ...
    'Box', 'off', ...
    'TextColor', 'k', ...
    'FontSize', 11);

% ===== 右上：hy_out 和 l_slag =====
ax2 = nexttile;
hold(ax2, 'on');
h = gobjects(6, 1);
for k = 1:3
    h(2*k-1) = plot(t_hy_plot, hy_out_plot(:, k), '-',  'LineWidth', 1.5, 'Color', phaseColor(k, :));
    h(2*k)   = plot(t_slag_plot, l_slag_plot(:, k), '--', 'LineWidth', 1.5, 'Color', phaseColor(k, :));
end
styleAxes(ax2);
xlim(ax2, [tMinPlot, inf]);
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('长度/高度 (m)', 'Color', 'k', 'FontSize', 12);
title('各相电弧长度 hy\_out 与泡沫渣高度 l\_slag', 'Color', 'k', 'FontSize', 13);
legend(ax2, h, ...
    {'A相 hy\_out', 'A相 l\_slag', ...
     'B相 hy\_out', 'B相 l\_slag', ...
     'C相 hy\_out', 'C相 l\_slag'}, ...
    'Location', 'eastoutside', ...
    'Box', 'off', ...
    'TextColor', 'k', ...
    'FontSize', 10, ...
    'Interpreter', 'tex');

% ===== 左中：FSI =====
ax3 = nexttile;
hold(ax3, 'on');
for k = 1:3
    plot(t_FSI_plot, FSI_plot(:, k), 'LineWidth', 1.5, 'Color', phaseColor(k, :));
end
styleAxes(ax3);
xlim(ax3, [tMinPlot, inf]);
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('FSI', 'Color', 'k', 'FontSize', 12);
title('各相 FSI', 'Color', 'k', 'FontSize', 13);
legend(ax3, phaseName, ...
    'Location', 'eastoutside', ...
    'Box', 'off', ...
    'TextColor', 'k', ...
    'FontSize', 11);

% ===== 右中：SampEn =====
ax4 = nexttile;
hold(ax4, 'on');
for k = 1:3
    plot(t_SampEn_plot, SampEn_plot(:, k), 'LineWidth', 1.5, 'Color', phaseColor(k, :));
end
styleAxes(ax4);
xlim(ax4, [tMinPlot, inf]);
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('SampEn', 'Color', 'k', 'FontSize', 12);
title('各相 SampEn', 'Color', 'k', 'FontSize', 13);
legend(ax4, phaseName, ...
    'Location', 'eastoutside', ...
    'Box', 'off', ...
    'TextColor', 'k', ...
    'FontSize', 11);

% ===== 左下：Z_adj(k) =====
ax5 = nexttile;
hold(ax5, 'on');
for k = 1:3
    plot(t_Zadjk_plot, Zadjk_plot(:, k), 'LineWidth', 1.5, 'Color', phaseColor(k, :));
end
styleAxes(ax5);
xlim(ax5, [tMinPlot, inf]);
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('Z\_adj(k)', 'Color', 'k', 'FontSize', 12, 'Interpreter', 'tex');
title('各相阻抗调整量 Z\_adj(k)', 'Color', 'k', 'FontSize', 13, 'Interpreter', 'tex');
legend(ax5, phaseName, ...
    'Location', 'eastoutside', ...
    'Box', 'off', ...
    'TextColor', 'k', ...
    'FontSize', 11);

% ===== 右下：R_real =====
ax6 = nexttile;
hold(ax6, 'on');
for k = 1:3
    plot(t_Rreal_plot, Rreal_plot(:, k), 'LineWidth', 1.5, 'Color', phaseColor(k, :));
end
styleAxes(ax6);
xlim(ax6, [tMinPlot, inf]);
xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
ylabel('R\_real (\Omega)', 'Color', 'k', 'FontSize', 12, 'Interpreter', 'tex');
title('各相阻抗 R\_real', 'Color', 'k', 'FontSize', 13, 'Interpreter', 'tex');
legend(ax6, phaseName, ...
    'Location', 'eastoutside', ...
    'Box', 'off', ...
    'TextColor', 'k', ...
    'FontSize', 11);

sgtitle('变阻抗控制系统关键变量', ...
    'Color', 'k', 'FontSize', 14);

%% Local functions
function [data, time] = getLoggedData(logs, nameCandidates)
    sig = [];
    for i = 1:numel(nameCandidates)
        sig = logs.getElement(nameCandidates{i});
        if ~isempty(sig)
            break;
        end
    end

    if isempty(sig)
        error('未找到 logged signal：%s', strjoin(nameCandidates, ', '));
    end

    vals = sig.Values;
    data = vals.Data;
    time = vals.Time;
end

function y = reshapeSignal(data, time)
    if isvector(data)
        y = data(:);
    else
        sz = size(data);
        Nt = numel(time);

        if sz(1) == Nt
            y = reshape(data, Nt, []);
        else
            order = [ndims(data), 1:ndims(data)-1];
            y = reshape(permute(data, order), Nt, []);
        end
    end
end

function styleAxes(ax)
    grid(ax, 'on');
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = [0.75 0.75 0.75];
    ax.GridAlpha = 0.35;
    ax.FontSize = 12;
    ax.LineWidth = 1;
end

function [t_ds, y_ds] = downsampleForPlot(t, y, maxPts)
    n = numel(t);

    if n <= maxPts
        t_ds = t;
        y_ds = y;
        return;
    end

    idx = round(linspace(1, n, maxPts));
    idx = unique(idx);

    t_ds = t(idx);
    y_ds = y(idx, :);
end