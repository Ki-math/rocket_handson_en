function plot_6dof_attitude_results(simOut, figureName)
%PLOT_6DOF_ATTITUDE_RESULTS  Plot 6DOF attitude angles and body angular rates.
%
%   Convert the model units rad / rad/s to deg / deg/s for display.
%   Expected logged signals: quaternionBI [-], pqr [rad/s].
%
%   Derive attitude from quaternionBI rather than the block's euler output.
%   Near a horizontal nose attitude, 321 Euler phi and psi become coupled.
%   At apogee and under parachute, phi may not represent physical roll;
%   replace it with nose-axis twist from quaternion swing-twist decomposition.
%   This matches the roll definition used by animate_flight.
%   Extract theta and psi from the DCM as before, for display only.
%
%   If quaternionBI is unavailable, use the original euler signal directly.
if nargin < 2
    figureName = "6DOF Attitude Result";
end
euler = get_attitude(simOut);
pqr   = get_matrix(simOut, "pqr");
if isempty(euler) && isempty(pqr)
    warning("plot_6dof_attitude_results:NoData", ...
"euler / pqr were not found in the logs.");
return
end
fig = figure("Name", figureName, "Color", "w", "Position", [150 150 1100 700]);
tiledlayout(fig, 2, 1, "TileSpacing", "compact", "Padding", "compact");
ax1 = nexttile;
hold(ax1, "on"); grid(ax1, "on");
plot_components(ax1, euler, ["\phi", "\theta", "\psi"]);
ylabel(ax1, "Euler [deg]");
title(ax1, figureName);
legend(ax1, "Location", "best");
ax2 = nexttile;
hold(ax2, "on"); grid(ax2, "on");
plot_components(ax2, pqr, ["p", "q", "r"]);
xlabel(ax2, "Time [s]");
ylabel(ax2, "Rate [deg/s]");
legend(ax2, "Location", "best");
end
function sig = get_attitude(simOut)
%GET_ATTITUDE  Return attitude angles [phi theta psi] in rad.
%   phi is nose-axis twist from quaternion swing-twist decomposition.
%   theta / psi are standard 321 Euler angles extracted from the DCM.
quat = get_matrix(simOut, "quaternionBI");
if isempty(quat)
    sig = get_matrix(simOut, "euler");
return
end
qData   = quat.data;
nSample = size(qData, 1);
angles  = zeros(nSample, 3);
qRef    = normalize_quaternion(qData(1, :));
for k = 1:nSample
    qk = normalize_quaternion(qData(k, :));
    % Decompose rotation relative to the reference attitude and retain the twist.
    qErr = quaternion_product(quaternion_conjugate(qRef), qk);
if qErr(1) < 0
        qErr = -qErr;   % Choose the shorter rotation representation.
end
    angles(k, 1) = 2 * atan2(qErr(2), qErr(1));
    % Derive theta / psi from the DCM. R maps "UEN <- body".
    R = quat_to_rotm(qk);
    angles(k, 2) = -asin(min(1, max(-1, R(3, 1))));
    angles(k, 3) = atan2(R(2, 1), R(1, 1));
end
angles(:, 1) = unwrap(angles(:, 1));
sig = struct("time", quat.time, "data", angles);
end
function R = quat_to_rotm(q)
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1 - 2*(y^2 + z^2), 2*(x*y - z*w),     2*(x*z + y*w);
     2*(x*y + z*w),     1 - 2*(x^2 + z^2), 2*(y*z - x*w);
     2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x^2 + y^2)];
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
function plot_components(ax, sig, names)
%PLOT_COMPONENTS  Convert a vector signal from rad to deg and plot each component.
if isempty(sig)
    text(ax, 0.5, 0.5, "no data", "Units", "normalized", ...
"HorizontalAlignment", "center", "Color", [0.6 0.6 0.6]);
return
end
for k = 1:min(numel(names), size(sig.data, 2))
    plot(ax, sig.time, rad2deg(sig.data(:, k)), ...
"LineWidth", 1.2, "DisplayName", names(k));
end
end
function sig = get_matrix(simOut, signalName)
%GET_MATRIX  Extract and reshape to [nSample x nComponent].
sig = [];
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
return
end
data = squeeze(ts.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(ts.Time)
    data = data.';
end
sig = struct("time", ts.Time(:), "data", data);
end