clc;clear;
% close all;

load("data/training_log/stage5/SimulationInfo302.mat");

logs = SimulationInfo.logsout;

ph = 2;

% Z_adj
Z_adj = logs.getElement("Z_adj");
Z_adj_t = Z_adj.Values.Time;
Z_adj_d = squeeze(Z_adj.Values.Data).';
Z_adj_d = Z_adj_d(:,ph);

% dZ
dZ = logs.getElement("dZ(k)");
dZ_t = dZ.Values.Time;
dZ_d = squeeze(dZ.Values.Data).';
dZ_d = dZ_d(:,ph);

% reward
reward = logs.getElement("reward");
reward_t = reward.Values.Time;
reward_d = squeeze(reward.Values.Data).';
reward_d = reward_d(:,ph);

% hy_out
hy_out = logs.getElement("hy_out");
hy_out_t = hy_out.Values.Time;
hy_out_d = squeeze(hy_out.Values.Data);
hy_out_d = hy_out_d(:,ph);

% l_slag
l_slag = logs.getElement("l_slag");
l_slag_t = l_slag.Values.Time;
l_slag_d = squeeze(l_slag.Values.Data);
l_slag_d = l_slag_d(:,ph);

% bRate
bRate = logs.getElement("bRate");
bRate_t = bRate.Values.Time;
bRate_d = squeeze(bRate.Values.Data).';
bRate_d = bRate_d(:,ph);

figure;
subplot(2,2,1);
plot(bRate_t, bRate_d);
xlabel("Time (s)");
ylabel("bRate (bps)");
legend("bRate");

subplot(2,2,2);
plot(hy_out_t, hy_out_d); hold on;
plot(l_slag_t, l_slag_d); hold off;
xlabel("Time (s)");
ylabel("hy_out");
legend("hy_out", "l_slag");

subplot(2,2,3);
plot(Z_adj_t, Z_adj_d); hold on;
plot(dZ_t, dZ_d); hold off;
xlabel("Time (s)");
ylabel("Z");
legend("Z_adj", "dZ(k)");

subplot(2,2,4);
plot(reward_t, reward_d);
xlabel("Time (s)");
ylabel("Reward");
legend("Reward");