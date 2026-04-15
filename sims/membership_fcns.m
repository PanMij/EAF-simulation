%% Draw membership functions of the variable impedance fuzzy supervisor
clear; clc; close all;

% Build FIS
fis = build_varimp_fis();

fig = figure( ...
    'Name', 'Membership Functions of VarImpSupervisor', ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

%% Input 1: FSI_n
ax1 = nexttile;
plotMFPanel(ax1, fis.Inputs(1), 'FSI\_n', '输入变量 FSI\_n 的隶属函数');

%% Input 2: SampEn_n
ax2 = nexttile;
plotMFPanel(ax2, fis.Inputs(2), 'SampEn\_n', '输入变量 SampEn\_n 的隶属函数');

%% Output: delta
ax3 = nexttile;
plotMFPanel(ax3, fis.Outputs(1), '\DeltaZ / Z_{init}', '输出变量 \delta 的隶属函数');

sgtitle('VarImpSupervisor 隶属函数', ...
    'Color', 'k', 'FontSize', 16, 'FontWeight', 'bold');

exportgraphics(fig, 'varimp_membership_functions_white.png', 'Resolution', 300);

%% Local function
function plotMFPanel(ax, varObj, xLabelText, titleText)
    hold(ax, 'on');

    xRange = varObj.Range;
    x = linspace(xRange(1), xRange(2), 1200);

    nMF = numel(varObj.MembershipFunctions);
    h = gobjects(nMF, 1);

    for i = 1:nMF
        mf = varObj.MembershipFunctions(i);
        y = evalmf(x, mf.Parameters, mf.Type);

        h(i) = plot(ax, x, y, 'LineWidth', 2.2);

        % Add dark labels near the peak of each membership function
        ymax = max(y);
        idx = find(abs(y - ymax) < 1e-12);
        xPeak = mean(x(idx));

        % Keep labels inside the axes
        yText = min(0.96, ymax + 0.04);

        text(ax, xPeak, yText, mf.Name, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'Color', 'k', ...
            'FontSize', 12, ...
            'FontWeight', 'bold');
    end

    grid(ax, 'on');
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = [0.75 0.75 0.75];
    ax.GridAlpha = 0.35;
    ax.FontSize = 12;
    ax.LineWidth = 1;
    ax.Box = 'on';

    xlim(ax, xRange);
    ylim(ax, [-0.05 1.08]);

    xlabel(xLabelText, 'Color', 'k', 'FontSize', 12, 'Interpreter', 'tex');
    ylabel('隶属度', 'Color', 'k', 'FontSize', 12);
    title(titleText, 'Color', 'k', 'FontSize', 14, 'FontWeight', 'bold');

    lgd = legend(ax, h, {varObj.MembershipFunctions.Name}, ...
        'Location', 'eastoutside', ...
        'Box', 'off', ...
        'TextColor', 'k', ...
        'FontSize', 11, ...
        'Interpreter', 'none');
    lgd.Color = 'w';
end