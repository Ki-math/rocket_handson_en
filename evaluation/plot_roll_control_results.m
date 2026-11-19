function plot_roll_control_results(simOut, figureName)
%PLOT_ROLL_CONTROL_RESULTS  Plot roll angle, roll rate, and fin deflection responses.
%
%   Convert the model units rad / rad/s to deg / deg/s for display.
%   Expected logged signals (no suffixes; angles and rates in radians):
%     phiCmd, delta, deltaCmd   [rad]
%     pCmd                      [rad/s]
%     quaternionBI              [-]     Used to derive roll angle
%     pqr                        [rad/s] First component is roll rate
%     The 1DOF model uses phi / p instead of quaternionBI / pqr.
%
%   phi is nose-axis twist derived from quaternionBI by swing-twist decomposition.
%   Avoid 321 Euler phi: near a horizontal nose attitude it couples with psi
%   and can spike at apogee. Use the same roll definition as animate_flight.
if nargin < 2
    figureName = "Roll Control Result";
end
phi      = get_roll_angle(simOut);
phiCmd   = pick_angle(simOut, "phiCmd", "",      1);
p        = pick_angle(simOut, "p",      "pqr",   1);
pCmd     = pick_angle(simOut, "pCmd",   "",      1);
delta    = pick_angle(simOut, "delta",  "",      1);
deltaCmd = pick_angle(simOut, "deltaCmd", "",    1);
fig = figure("Name", figureName, "Color", "w", "Position", [120 120 1100 780]);
tiledlayout(fig, 3, 1, "TileSpacing", "compact", "Padding", "compact");
ax1 = nexttile;
hold(ax1, "on"); grid(ax1, "on");
plot_deg(ax1, phi,    "-",  1.4, "\phi");
plot_deg(ax1, phiCmd, "--", 1.2, "\phi cmd");
ylabel(ax1, "\phi [deg]");
title(ax1, figureName);
legend(ax1, "Location", "best");
ax2 = nexttile;
hold(ax2, "on"); grid(ax2, "on");
plot_deg(ax2, p,    "-",  1.4, "p");
plot_deg(ax2, pCmd, "--", 1.2, "p cmd");
ylabel(ax2, "p [deg/s]");
legend(ax2, "Location", "best");
ax3 = nexttile;
hold(ax3, "on"); grid(ax3, "on");
plot_deg(ax3, delta,    "-",  1.4, "\delta_f");
plot_deg(ax3, deltaCmd, "--", 1.2, "\delta cmd");
xlabel(ax3, "Time [s]");
ylabel(ax3, "\delta_f [deg]");
legend(ax3, "Location", "best");
end
function sig = get_roll_angle(simOut)
%GET_ROLL_ANGLE  Return roll angle in rad, using quaternions when available.
ts = get_timeseries(simOut, "quaternionBI");
if isempty(ts)
    sig = pick_angle(simOut, "phi", "euler", 1);   % For the 1DOF model
return
end
qData = squeeze(ts.Data);
if size(qData, 1) ~= numel(ts.Time)
    qData = qData.';
end
nSample = size(qData, 1);
roll    = zeros(nSample, 1);
qRef    = normalize_quaternion(qData(1, :));
for k = 1:nSample
    % Decompose rotation relative to the reference attitude and retain the twist.
    qErr = quaternion_product(quaternion_conjugate(qRef), normalize_quaternion(qData(k, :)));
if qErr(1) < 0
        qErr = -qErr;   % Choose the shorter rotation representation.
end
    roll(k) = 2 * atan2(qErr(2), qErr(1));
end
sig = struct("time", ts.Time(:), "data", unwrap(roll));
end
function q = normalize_quaternion(q)
q = reshape(double(q), 1, 4);
magnitude = norm(q);
if magnitude < eps
    q = [1 0 0 0];
else
    q = q / magnitude;
end
end
function q = quaternion_conjugate(q)
q = [q(1), -q(2), -q(3), -q(4)];
end
function q = quaternion_product(a, b)
q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end
function plot_deg(ax, sig, style, width, name)
%PLOT_DEG  Convert logged angles from rad to deg and plot them.
if isempty(sig)
return
end
plot(ax, sig.time, rad2deg(sig.data), style, "LineWidth", width, "DisplayName", name);
end
function sig = pick_angle(simOut, primaryName, vectorName, component)
%PICK_ANGLE  Prefer a scalar signal; otherwise extract the requested vector component.
sig = get_scalar(simOut, primaryName);
if ~isempty(sig) || strlength(vectorName) == 0
return
end
sig = get_component(simOut, vectorName, component);
end
function sig = get_scalar(simOut, signalName)
sig = [];
ts = get_timeseries(simOut, signalName);
if isempty(ts)
return
end
data = squeeze(ts.Data);
sig = struct("time", ts.Time(:), "data", data(:));
end
function sig = get_component(simOut, signalName, component)
sig = [];
ts = get_timeseries(simOut, signalName);
if isempty(ts)
return
end
data = squeeze(ts.Data);
if isvector(data)
    column = data(:);
elseif size(data, 2) >= component
    column = data(:, component);
else
    column = data(component, :).';
end
sig = struct("time", ts.Time(:), "data", column);
end
function ts = get_timeseries(simOut, signalName)
ts = [];
try
    names = string(simOut.logsout.getElementNames);
catch
return
end
if ~any(names == string(signalName))
return
end
try
    ts = simOut.logsout.get(char(signalName)).Values;
catch
end
end