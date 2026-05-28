% aeb.m
% -------------------------------------------------------------------------
% Automatic Emergency Braking (Time-To-Collision)
% A noisy forward sensor reports range to an obstacle; the controller
% computes Time-To-Collision (TTC) and escalates through forward-collision
% warning -> partial brake -> full brake. Reports the final stopping gap.
%
% Run:  >> aeb
%
% Author: Ujjwal Singla
% License: MIT  |  Requires: base MATLAB (no toolboxes)
% -------------------------------------------------------------------------

clear; clc; close all;
rng(1);

%% Thresholds (seconds of TTC) and brake levels
TTC_WARN = 2.6; TTC_PARTIAL = 2.0; TTC_FULL = 1.4;
A_PARTIAL = -4.0; A_FULL = -8.0;

dt = 0.02; v0 = 14.0; obstacle = 40.0; sensor_noise = 0.15;
n = round(8.0/dt);
v = v0; x = 0;
state = 'CRUISE';
t = zeros(n,1); vlog = zeros(n,1); rng_log = zeros(n,1); ttc_log = zeros(n,1);
kmax = n;

for k = 1:n
    true_range = obstacle - x;
    if true_range <= 0, state = 'COLLISION'; kmax = k-1; break; end
    meas_range = max(0.01, true_range + sensor_noise*randn);
    if v > 0.1, ttc = meas_range / v; else, ttc = inf; end

    if v <= 0.05
        state = 'STOPPED'; a = 0;
    elseif ttc < TTC_FULL
        state = 'FULL_BRAKE'; a = A_FULL;
    elseif ttc < TTC_PARTIAL
        state = 'PARTIAL_BRAKE'; a = A_PARTIAL;
    elseif ttc < TTC_WARN
        state = 'WARN'; a = 0;
    else
        state = 'CRUISE'; a = 0;
    end

    v = max(0, v + a*dt);
    x = x + v*dt;
    t(k) = (k-1)*dt; vlog(k) = v; rng_log(k) = true_range; ttc_log(k) = ttc;
end

idx = 1:kmax;
stop_gap = obstacle - x;
collision = strcmp(state,'COLLISION');
fprintf('stop gap: %.2f m   collision: %d\n', stop_gap, collision);

figure('Color','w','Position',[100 100 760 640]);
subplot(3,1,1); plot(t(idx), vlog(idx)*3.6,'LineWidth',1.4); grid on;
ylabel('Speed (km/h)');
title(sprintf(['AEB (TTC)  |  stopped %.2f m short  |  collision: %d'], ...
    stop_gap, collision));
subplot(3,1,2); plot(t(idx), rng_log(idx),'LineWidth',1.4,'Color',[0.18 0.63 0.18]);
grid on; ylabel('Range (m)');
subplot(3,1,3); plot(t(idx), min(ttc_log(idx),6),'LineWidth',1.4,'Color',[0.85 0.16 0.16]);
hold on; plot(t(idx), TTC_WARN+0*t(idx),'--'); plot(t(idx), TTC_PARTIAL+0*t(idx),'--'); plot(t(idx), TTC_FULL+0*t(idx),'--');
grid on; ylabel('TTC (s)'); xlabel('Time (s)');
try, print(gcf,'demo.png','-dpng','-r110'); catch, end
