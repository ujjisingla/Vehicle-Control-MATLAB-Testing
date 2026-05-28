% soc_ekf.m
% -------------------------------------------------------------------------
% Battery SOC Estimation via Extended Kalman Filter
% Estimates lithium-ion State of Charge from noisy terminal-voltage and
% current using a 1st-order RC equivalent-circuit model and an EKF.
%
% State:   x = [SOC; V_rc]
% Measure: y = OCV(SOC) - I*R0 - V_rc
%
% Run:  >> soc_ekf
%
% Author: Ujjwal Singla
% License: MIT  |  Requires: base MATLAB (no toolboxes)
% -------------------------------------------------------------------------

function soc_ekf()
close all;
rng(0);

%% Equivalent-circuit parameters (18650-class cell)
Q_Ah = 3.0; R0 = 0.015; R1 = 0.020; C1 = 2000;
dt = 1.0; n = 1200;
cap_s = Q_Ah * 3600;
a_rc  = exp(-dt / (R1*C1));

%% Current profile (A, + = discharge)
I = zeros(n,1);
I(100:400) = 2.0; I(500:700) = 4.0; I(800:1100) = 1.5;

%% Truth model -> clean + noisy measured voltage
soc_true = zeros(n,1); v_clean = zeros(n,1);
soc = 0.9; vrc = 0;
for k = 1:n
    soc = soc - I(k)*dt/cap_s;
    vrc = a_rc*vrc + R1*(1-a_rc)*I(k);
    soc_true(k) = soc;
    v_clean(k)  = ocv(soc) - I(k)*R0 - vrc;
end
v_meas = v_clean + 0.02*randn(n,1);

%% EKF (deliberately wrong initial SOC = 0.6)
x = [0.6; 0];
P = diag([0.05, 1e-3]);
Qn = diag([1e-7, 1e-6]);
Rn = 2e-3;
soc_est = zeros(n,1);
for k = 1:n
    Ik = I(k);
    % predict
    x(1) = x(1) - Ik*dt/cap_s;
    x(2) = a_rc*x(2) + R1*(1-a_rc)*Ik;
    F = [1 0; 0 a_rc];
    P = F*P*F' + Qn;
    % update
    y_pred = ocv(x(1)) - Ik*R0 - x(2);
    H = [docv_dsoc(x(1)), -1];
    S = H*P*H' + Rn;
    K = (P*H')/S;
    x = x + K*(v_meas(k) - y_pred);
    x(1) = min(max(x(1),0),1);
    P = (eye(2) - K*H)*P;
    soc_est(k) = x(1);
end

rmse = sqrt(mean((soc_est(50:end) - soc_true(50:end)).^2)) * 100;
fprintf('Final true SOC: %.1f%%   EKF: %.1f%%   RMSE: %.2f%%\n', ...
    soc_true(end)*100, soc_est(end)*100, rmse);

%% Plot
t = (0:n-1)' * dt;
figure('Color','w','Position',[100 100 760 560]);
subplot(2,1,1);
plot(t, soc_true*100,'k','LineWidth',2); hold on;
plot(t, soc_est*100,'--','LineWidth',1.6,'Color',[0.85 0.16 0.16]);
grid on; ylabel('SOC (%)'); legend('True SOC','EKF estimate');
title(sprintf('Battery SOC EKF  |  converges 60%%->truth  |  RMSE %.2f%%', rmse));
subplot(2,1,2);
plot(t, v_meas,'Color',[0.73 0.73 0.73]); hold on;
plot(t, v_clean,'LineWidth',1.2,'Color',[0.12 0.47 0.71]);
grid on; ylabel('Voltage (V)'); xlabel('Time (s)');
legend('Measured V (noisy)','True terminal V');
try, print(gcf,'demo.png','-dpng','-r110'); catch, end

end

%% ---- helpers ----
function v = ocv(soc)
    soc = min(max(soc,0),1);
    v = 3.0 + 0.9*soc - 0.30*exp(-12*soc) + 0.18*soc.^2 ...
        + 0.05*tanh(8*(soc-0.5));
end
function d = docv_dsoc(soc)
    h = 1e-4;
    d = (ocv(soc+h) - ocv(soc-h)) / (2*h);
end
