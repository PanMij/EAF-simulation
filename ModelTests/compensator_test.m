clc; clear;

%% Find the voltage-speed characteristics of the hydraulic system
mdl = 'hydraulic_system_characteristic';
volSteps = -1.2:0.001:1.2;        % input voltage steps
nVolSteps = length(volSteps);
tKeep = 4;                   % time for keeping each step
tStep = 1;                % simulation time step
t = 0:tStep:nVolSteps * tKeep;    % time vector
u = zeros(length(t), 1);
for i = 0 : nVolSteps - 1
    u(i * tKeep / tStep + 1 : (i + 1) * tKeep / tStep) = volSteps(i + 1);
end
u(end) = volSteps(end);
simin = timeseries(u, t');  % input voltage time series

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(nVolSteps * tKeep));
in = in.setModelParameter('SimulationMode', 'rapid-accelerator');
in = in.setModelParameter('SaveTime', 'off');
in = in.setVariable('L_ub', inf);
in = in.setVariable('L_lb', -inf);

out = sim(in, 'ShowProgress', 'on');
tv = out.yout.getElement('v').Values.Time;
v = out.yout.getElement('v').Values.Data;
tOp = out.yout.getElement('op').Values.Time;
op = out.yout.getElement('op').Values.Data;
% tl = out.yout.getElement('l').Values.Time;
% l = out.yout.getElement('l').Values.Data;

% figure;
% subplot(2,1,1);
% plot(t, u, 'LineWidth', 1.5);
% title('Input Voltage');
% subplot(2,1,2);
% plot(tv, v, 'LineWidth', 1.5);
% title('Load speed');
% subplot(2,2,3);
% plot(tOp, op, 'LineWidth', 1.5);
% title('valve opening');
% subplot(2,2,4);
% plot(tl, l, 'LineWidth', 1.5);
% title('Load position');

%% Build LUT using steady-state speed within each voltage step
tSettle = 0.6 * tKeep;  % ignore initial transient in each step
minSamples = 5;
v_ss = nan(nVolSteps, 1);

for i = 1:nVolSteps
    t0 = (i - 1) * tKeep;
    t1 = i * tKeep;
    mask = tv >= (t0 + tSettle) & tv < t1;
    if nnz(mask) < minSamples
        mask = tv >= (t0 + 0.8 * tKeep) & tv < t1;
    end
    if nnz(mask) > 0
        v_ss(i) = mean(v(mask));
    end
end

valid = ~isnan(v_ss);
lut_speed = v_ss(valid);
lut_voltage = volSteps(valid).';

% Ensure monotonic lookup in speed domain
[lut_speed, sortIdx] = sort(lut_speed);
lut_voltage = lut_voltage(sortIdx);

% Collapse duplicate/near-duplicate speeds to one-to-one mapping
% - Boundary stacks: pick value closest to the monotonic part
% - Interior stacks: use mean value
speedTol = 1e-4;
speedBins = round(lut_speed / speedTol) * speedTol;
[uniqSpeed, ~, binIdx] = unique(speedBins);
uniqVoltage = nan(size(uniqSpeed));
groupSizes = accumarray(binIdx, 1);

for g = 1:numel(uniqSpeed)
    members = (binIdx == g);
    vGroup = lut_voltage(members);

    if groupSizes(g) == 1
        uniqVoltage(g) = vGroup;
        continue
    end

    isBoundary = (g == 1) || (g == numel(uniqSpeed));
    if isBoundary
        if g == 1
            neighborTarget = mean(lut_voltage(binIdx == g + 1));
        else
            neighborTarget = mean(lut_voltage(binIdx == g - 1));
        end
        [~, pickIdx] = min(abs(vGroup - neighborTarget));
        uniqVoltage(g) = vGroup(pickIdx);
    else
        uniqVoltage(g) = mean(vGroup);
    end
end

lut_speed = uniqSpeed;
lut_voltage = uniqVoltage;

figure;
plot(lut_speed, lut_voltage, 'o-');
xlabel('Load speed');
ylabel('Input voltage');
title('Steady-state voltage-speed LUT');

% Save LUT
outDir = fullfile(pwd, "data");
if ~exist(outDir, "dir"), mkdir(outDir); end
save(fullfile(outDir, "voltage_speed_lut.mat"), "lut_speed", "lut_voltage");
