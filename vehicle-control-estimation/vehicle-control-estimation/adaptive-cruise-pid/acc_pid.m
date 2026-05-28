% acc_pid.m
% -------------------------------------------------------------------------
% Adaptive Cruise Control (PID)
% Ego vehicle follows a lead vehicle at a constant time-gap using a PID
% controller with integral anti-windup, acceleration limits, and a jerk
% constraint. Falls back to set-speed control when the gap is large.
%
% Run:  >> acc_pid
%
% Author: Ujjwal Singla
% License: MIT  |  Requires: base MATLAB (no toolboxes)
% -------------------------------------------------------------------------

clear; clc; close all;

dt = 0.1; T = 60; n = round(T/dt);
t = (0:n-1)' * dt;
time_gap = 1.5; set_speed = 30;

%% Lead-vehicle profile
v_lead = 25 * ones(n,1);
v_lead(round(15/dt):round(30/dt)) = 15;     % lead slows
v_lead(round(40/dt):end) = 28;              % lead speeds up
x_lead = cumsum(v_lead)*dt + 40;            % starts 40 m ahead

%% PID gains and limits
Kp = 0.45; Ki = 0.05; Kd = 0.12;
a_lo = -3.5; a_hi = 2.5; jerk_lim = 3.0;

x_ego = zeros(n,1); v_ego = zeros(n,1); a_ego = zeros(n,1);
v_ego(1) = 25;
I = 0; e_prev = 0; a_prev = 0;

for k = 2:n
    gap     = x_lead(k-1) - x_ego(k-1);
    des_gap = time_gap*v_ego(k-1) + 5;
    err     = gap - des_gap;
    % PID with anti-windup
    I = I + err*dt;
    d = (err - e_prev)/dt; e_prev = err;
    u = Kp*err + Ki*I + Kd*d;
    u_sat = min(max(u, a_lo), a_hi);
    if u ~= u_sat, I = I - err*dt; end       % clamp integral
    a_gap = u_sat;
    a_spd = 0.6*(set_speed - v_ego(k-1));    % set-speed term
    a_cmd = min(a_gap, a_spd);               % take the safer (smaller)
    % jerk limit
    a_cmd = min(max(a_cmd, a_prev - jerk_lim*dt), a_prev + jerk_lim*dt);
    a_prev = a_cmd;
    v_ego(k) = max(0, v_ego(k-1) + a_cmd*dt);
    x_ego(k) = x_ego(k-1) + v_ego(k)*dt;
    a_ego(k) = a_cmd;
end

gap = x_lead - x_ego;
fprintf('min gap: %.1f m  (stayed safe: %d)\n', min(gap), min(gap) > 0);

figure('Color','w','Position',[100 100 760 640]);
subplot(3,1,1);
plot(t, v_lead*3.6,'Color',[0.5 0.5 0.5],'LineWidth',1.4); hold on;
plot(t, v_ego*3.6,'LineWidth',1.4,'Color',[0.12 0.47 0.71]); grid on;
ylabel('Speed (km/h)'); legend('Lead','Ego (ACC)');
title('Adaptive Cruise Control (PID) - gap-keeping with jerk limit');
subplot(3,1,2); plot(t, gap,'LineWidth',1.4,'Color',[0.18 0.63 0.18]);
grid on; ylabel('Gap (m)');
subplot(3,1,3); plot(t, a_ego,'LineWidth',1.4,'Color',[0.85 0.16 0.16]);
grid on; hold on; plot(t, 0*t, 'k'); ylabel('Accel (m/s^2)'); xlabel('Time (s)');
try, print(gcf,'demo.png','-dpng','-r110'); catch, end
