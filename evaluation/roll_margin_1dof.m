%% Roll-control stability margins (1DOF)
% Derive roll-axis dynamics analytically and evaluate open-loop stability margins.
% Keep the equations explicit instead of linearizing the Simulink model.
%
% Prerequisite: define the roll-design parameters before running this script.

%% 1) Roll-axis equations of motion
% Treat the vehicle as a single degree of freedom about the roll axis.
%
%     Ixx * dp/dt = Lp * p + Ldelta * delta
%     dphi/dt     = p
%
%   p      : Roll rate       [rad/s]
%   phi    : Roll angle      [rad]
%   delta  : Fin deflection  [rad]
%   Lp     : Roll damping    [Nm/(rad/s)]  Negative: opposes rotation
%   Ldelta : Control effectiveness [Nm/rad] Positive: deflection produces rotation
%
% Lp and Ldelta scale with dynamic pressure; evaluate at roll_design_qbar.

Ixx    = roll_design_Ixx;
Lp     = roll_Lp;
Ldelta = roll_Ldelta;

s = tf("s");

% Fin deflection -> roll rate
P_p   = Ldelta / (Ixx * s - Lp);

% Roll rate -> roll angle (pure integration)
P_phi = 1 / s;

fprintf("--- Uncontrolled roll-axis dynamics ---\n");
fprintf("  Time constant Ixx/|Lp| = %.2f s\n", Ixx / abs(Lp));
fprintf("  Steady-state gain Ldelta/|Lp| = %.1f (rad/s)/rad = %.2f (deg/s)/deg\n", ...
    Ldelta / abs(Lp), Ldelta / abs(Lp));

%% 2) Actuator and sensors
% First-order lags introduce phase lag and strongly affect stability margins.
A     = 1 / (fin_actuatorTau   * s + 1);   % Fin actuator
G_p   = 1 / (sensor_gyroTau    * s + 1);   % Rate gyro
G_phi = 1 / (sensor_attitudeTau * s + 1);  % Attitude sensor

%% 3) Control law
% Inner loop: proportional roll-rate control
% Outer loop: proportional roll-angle control
C_in  = controller_inner_Kp;
K_out = controller_outer_Kp;

%% 4) Open-loop transfer functions
% --- 4a) Loop opened at the fin command ---------------------------------
% Follow the path from delta_cmd around the loop back to delta_cmd.
%
%   delta   = A * delta_cmd
%   p       = P_p * delta,      phi = P_phi * p
%   pHat    = G_p * p,          phiHat = G_phi * phi
%   pCmd    = K_out * (phiCmd - phiHat)
%   delta_cmd = C_in * (pCmd - pHat)
%
% Set phiCmd = 0 and collect the return contributions.
%
%   L_fin = C_in * A * P_p * ( K_out * G_phi * P_phi + G_p )
L_fin = C_in * A * P_p * (K_out * G_phi * P_phi + G_p);

% --- 4b) Loop opened at the rate command (inner loop remains closed) ------
% Inner closed loop: p / pCmd = C_in*A*P_p / (1 + C_in*A*P_p*G_p)
T_in  = feedback(C_in * A * P_p, G_p);
L_out = K_out * G_phi * P_phi * T_in;

%% 5) Stability margins
loopNames = ["Fin command loop", "Outer roll loop"];
loops     = {L_fin, L_out};

fprintf("\n--- Stability margins ---\n");
fprintf("  %-18s %8s %9s %11s\n", "Loop", "GM[dB]", "PM[deg]", "wc[rad/s]");
for k = 1:numel(loops)
    [gm, pm, ~, wc] = margin(loops{k});
    fprintf("  %-18s %8.2f %9.2f %11.3f\n", loopNames(k), 20*log10(gm), pm, wc);
end

%% 6) Closed-loop response
T_phi = feedback(K_out * P_phi * T_in, G_phi);   % phiCmd -> phi
fprintf("\n--- Closed loop (phiCmd -> phi) ---\n");
info = stepinfo(T_phi);
fprintf("  Rise time %.2f s / settling time %.2f s / overshoot %.1f %%\n", ...
    info.RiseTime, info.SettlingTime, info.Overshoot);
fprintf("  Bandwidth %.2f rad/s\n", bandwidth(T_phi));

%% 7) Plots
figure("Name", "1DOF Roll Stability Margin", "Color", "w", "Position", [120 120 1000 720]);
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
margin(L_fin); grid on; title("Fin command loop");

nexttile;
margin(L_out); grid on; title("Outer roll loop");

nexttile;
nichols(L_fin, L_out); grid on;
legend(loopNames, "Location", "best"); title("Nichols plot");

nexttile;
step(T_phi, 5); grid on; title("Roll-angle step response");

%% 8) Effect of gain changes on stability margins
fprintf("\n--- Fin command loop with varying inner-loop gain ---\n");
fprintf("  %-10s %8s %9s %11s\n", "inner_Kp", "GM[dB]", "PM[deg]", "wc[rad/s]");
for kp = controller_inner_Kp * [0.5 1 2 4]
    Ck = kp;
    Lk = Ck * A * P_p * (K_out * G_phi * P_phi + G_p);
    [gm, pm, ~, wc] = margin(Lk);
    fprintf("  %-10.3f %8.2f %9.2f %11.3f\n", kp, 20*log10(gm), pm, wc);
end
