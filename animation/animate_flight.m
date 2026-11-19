function animate_flight(data, varargin)
%ANIMATE_FLIGHT  Animate a 6DOF rocket flight (UEN reference with separate coordinate frames).
%
%   ANIMATE_FLIGHT(SIMOUT) animates the flight recorded in the Simulink
%   output SIMOUT. ANIMATE_FLIGHT(LOGSTRUCT) also accepts a struct
%   containing fields with the same names.
%
%   Coordinate conventions
%     Inertial frame: UEN (x = Up, y = East, z = North). Vertical attitude
%                     is theta = 0, avoiding the 321 Euler singularity
%                     during vertical ascent. The singularity is at a horizontal nose attitude.
%     Body axes: xb = U, yb = E, zb = N at zero attitude.
%     Display: MATLAB 3D axes conventionally use Z up, so coordinates
%              are reordered internally to ENU for plotting.
%
%   Required logs
%     positionUEN : Xe output of the 6DOF block (U, E, N order)
%     quaternionBI  : q output of the 6DOF (Quaternion) block
%                     （[q0 q1 q2 q3]、quat2dcm(q) = DCM_body_from_UEN）
%   Optional logs
%     velocityBody, pqr, euler, delta,
%     phiCmd, pCmd, deltaCmd   (all in rad or rad/s)
%
%   ANIMATE_FLIGHT(..., NAME, VALUE) options
%     PlaybackSpeed (8)        Playback speed relative to real time
%     FrameRate     (30)       Playback frame rate [fps]. Logs are uniformly
%                             resampled at this interval, keeping frame
%                             spacing independent of solver steps and log decimation.
%     BodyLength    (2.8)      Vehicle length [m]
%     CgFraction    (0.5)      Position reference on body (0 = tail, 1 = nose)
%     WorldScale    ([])       Vehicle scale in world view ([] = automatic)
%     Trail         (true)     Show or hide the trajectory
%     Layout        ("full")   "full"      : world view + frames + time histories
%                             "worldonly" : world view only
%     WorldView     ([40 18])  World-view [azimuth elevation] in deg
%     FrameView     ([42 20])  Frame-panel [azimuth elevation] in deg
%     ShowTrends    (true)     Show the time-history panel
%     SaveVideo     (false)
%     VideoFile     ("flight_animation.mp4")
%
%   Screen layout (Layout = "full")
%     Left: UEN world view, trajectory, ground, drop line, ground shadow, ENU triad
%     Upper right: inertial axes (gray), body axes Xb/Yb/Zb (red/green/blue), readout
%     Lower right: altitude / speed / dynamic pressure histories and playback cursor
%
%   Roll angle
%     Displayed roll is the twist about the nose axis relative to the
%     initial attitude, obtained by quaternion swing-twist decomposition, avoiding the 321 Euler phi singularity.
%
%   Example
%     out = sim("rocket_6dof_with_roll_controller");
%     animate_flight(out, "BodyLength", 2.8, "PlaybackSpeed", 5);
%     animate_flight(out, "Layout", "worldonly");

opts = parse_inputs(varargin{:});
[logs, logsFull] = prepare_logs(data, opts);
ui   = build_ui(logs, logsFull, opts);
run_animation(ui, logs, opts);
end


%% ======================================================================
%  Input parsing
%  ======================================================================
function opts = parse_inputs(varargin)
isPosScalar = @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0;
isFlag      = @(x) (islogical(x) || isnumeric(x)) && isscalar(x);
isPair      = @(x) isnumeric(x) && numel(x) == 2;

parser = inputParser;
parser.FunctionName = "animate_flight";
parser.PartialMatching = false;   % Reject abbreviated names to catch option-name mistakes.
parser.addParameter("PlaybackSpeed", 8,        isPosScalar);
parser.addParameter("FrameRate",     30,       isPosScalar);
parser.addParameter("BodyLength",    2.8,      isPosScalar);
parser.addParameter("CgFraction",    0.5,      @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
parser.addParameter("ChuteDiameter", 2.0,      isPosScalar);
parser.addParameter("ChuteRiser",    3.0,      isPosScalar);   % Attachment-to-canopy distance [m]
parser.addParameter("WorldScale",    [],       @(x) isempty(x) || isPosScalar(x));
parser.addParameter("Trail",         true,     isFlag);
parser.addParameter("Layout",        "full",   @(x) any(strcmpi(x, ["full", "worldonly"])));
parser.addParameter("WorldView",     [40 18],  isPair);
parser.addParameter("FrameView",     [42 20],  isPair);
parser.addParameter("ShowTrends",    true,     isFlag);
parser.addParameter("SaveVideo",     false,    isFlag);
parser.addParameter("VideoFile",     "flight_animation.mp4", @(x) isstring(x) || ischar(x));
parser.parse(varargin{:});

opts = parser.Results;
opts.Layout     = string(lower(opts.Layout));
opts.VideoFile  = string(opts.VideoFile);
opts.Trail      = logical(opts.Trail);
opts.ShowTrends = logical(opts.ShowTrends);
opts.SaveVideo  = logical(opts.SaveVideo);
end


%% ======================================================================
%  Log preparation
%  ======================================================================
function [logs, logsFull] = prepare_logs(data, opts)
logs = extract_logs(data);

if isempty(logs.t) || isempty(logs.positionUEN) || isempty(logs.quaternionBI)
    error("animate_flight:MissingLogs", ...
        "positionUEN and quaternionBI are required (check that logsout contains them).");
end

% Animate decimated logs, but retain the original resolution for time histories.
% Also create and return logsFull before decimation.
logsFull = derive_all(logs);
logs     = derive_all(resample_logs(logs, opts));
end


function logs = derive_all(logs)
%DERIVE_ALL  Compute derived quantities used for display.
logs = derive_display_position(logs);
logs = derive_display_units(logs);
logs = derive_extras(logs);
logs = derive_attitude(logs);
end

function logs = extract_logs(data)
scalarNames = ["phiCmd", "pCmd", "deltaCmd", "delta", "qbar", "Mach", "chuteOpen"];
vectorNames = ["positionUEN", "velocityBody", "quaternionBI", "euler", "pqr"];

logs = struct("t", []);
for name = [vectorNames, scalarNames]
    logs.(name) = [];
end

if isstruct(data)
    logs = merge_structs(logs, data);
    if isempty(logs.t) && ~isempty(logs.positionUEN)
        logs.t = 0:size(logs.positionUEN, 2) - 1;
    end
    return
end

% Signals inside enabled subsystems have no samples before activation,
% so align these shorter logs to the reference time vector before use.
for name = vectorNames
    sig = get_logged_signal(data, name);
    if isempty(sig)
        continue
    end
    logs.(name) = to_columns(sig.Data);
    if isempty(logs.t)
        logs.t = sig.Time(:).';
    end
    logs.(name) = align_to_reference_time(logs.(name), sig.Time(:).', logs.t);
end

for name = scalarNames
    sig = get_logged_signal(data, name);
    if ~isempty(sig)
        logs.(name) = align_to_reference_time(sig.Data(:).', sig.Time(:).', logs.t);
    end
end

% Provide component aliases for convenient downstream access.
logs = split_triplet(logs, "euler", ["phi", "theta", "psi"]);
logs = split_triplet(logs, "pqr",   ["p", "q", "r"]);
end

function value = align_to_reference_time(value, tSource, tRef)
%ALIGN_TO_REFERENCE_TIME  Align a signal to the reference time vector.
%   Extrapolate short logs with endpoint values (first value before activation).
if isempty(value) || isempty(tRef) || numel(tSource) == numel(tRef)
    return
end
if numel(tSource) < 2
    value = repmat(value(:, 1), 1, numel(tRef));
    return
end
tClamped = min(max(tRef, tSource(1)), tSource(end));
value = interp1(tSource, value.', tClamped, "linear").';
end


function sig = get_logged_signal(out, name)
sig = [];
try
    sig = out.logsout.get(name).Values;
    return
catch
end
try
    sig = out.(name);
catch
end
end

function logs = split_triplet(logs, sourceName, targetNames)
source = logs.(sourceName);
for k = 1:numel(targetNames)
    if size(source, 1) >= k
        logs.(targetNames(k)) = source(k, :);
    else
        logs.(targetNames(k)) = [];
    end
end
end

function out = to_columns(data)
% Reshape to [nComponent x nSample], with one column per time sample.
data = squeeze(data);
if isvector(data)
    out = data(:).';
elseif size(data, 1) >= size(data, 2)
    out = data.';
else
    out = data;
end
end

function logs = resample_logs(logs, opts)
%RESAMPLE_LOGS  Resample logs onto a uniform playback time grid.
%   Decouple frame spacing from solver steps and log decimation so that
%   playback smoothness is determined by FrameRate alone.
tSource  = logs.t;
duration = tSource(end) - tSource(1);
if numel(tSource) < 2 || duration <= 0
    return
end

nFrame = max(2, round(duration / opts.PlaybackSpeed * opts.FrameRate));
tFrame = linspace(tSource(1), tSource(end), nFrame);

% Align quaternion signs before interpolation. Linear interpolation across
% opposite representations of the same rotation would cause attitude jumps.
logs.quaternionBI = align_quaternion_signs(logs.quaternionBI);

for name = string(fieldnames(logs)).'
    value = logs.(name);
    if isempty(value) || name == "t" || size(value, 2) ~= numel(tSource)
        continue
    end
    logs.(name) = interp1(tSource, value.', tFrame).';
end

logs.t = tFrame;
logs.quaternionBI = logs.quaternionBI ./ vecnorm(logs.quaternionBI, 2, 1);
end

function q = align_quaternion_signs(q)
for k = 2:size(q, 2)
    if dot(q(:, k), q(:, k - 1)) < 0
        q(:, k) = -q(:, k);
    end
end
end

function logs = derive_display_position(logs)
% Logs use UEN; reorder to ENU for MATLAB 3D axes with Z pointing up.
logs.position_m = enu_from_uen() * logs.positionUEN;
end

function logs = derive_display_units(logs)
%DERIVE_DISPLAY_UNITS  Convert model rad / rad/s values to deg / deg/s for display.
radToDeg = ["phi", "theta", "psi", "phiCmd", "delta", "deltaCmd"];
degName  = ["phi_deg", "theta_deg", "psi_deg", "phiCmd_deg", "delta_deg", "deltaCmd_deg"];
for k = 1:numel(radToDeg)
    if has_log(logs, radToDeg(k))
        logs.(degName(k)) = rad2deg(logs.(radToDeg(k)));
    else
        logs.(degName(k)) = [];
    end
end

rateToDps = ["p", "q", "r", "pCmd"];
dpsName   = ["p_dps", "q_dps", "r_dps", "pCmd_dps"];
for k = 1:numel(rateToDps)
    if has_log(logs, rateToDps(k))
        logs.(dpsName(k)) = rad2deg(logs.(rateToDps(k)));
    else
        logs.(dpsName(k)) = [];
    end
end
end

function logs = derive_extras(logs)
logs.altitude_m = logs.position_m(3, :);
logs.east_m = logs.position_m(1, :);
logs.north_m = logs.position_m(2, :);
logs.range_m = hypot(logs.east_m, logs.north_m);
if ~isempty(logs.velocityBody)
    logs.speed_mps = vecnorm(logs.velocityBody, 2, 1);
else
    logs.speed_mps = [];
end
end

function logs = derive_attitude(logs)
%DERIVE_ATTITUDE  Derive all displayed attitude quantities from quaternions.
%   Avoid euler_deg: near the 321 Euler singularity, phi and psi become
%   coupled and can misrepresent the actual motion when displayed directly.
nSample = numel(logs.t);
qRef    = normalize_quaternion(logs.quaternionBI(:, 1));

logs.rollNose_deg = zeros(1, nSample);
logs.swing_deg    = zeros(1, nSample);
logs.noseElev_deg = zeros(1, nSample);
logs.noseAzim_deg = zeros(1, nSample);

for k = 1:nSample
    q = normalize_quaternion(logs.quaternionBI(:, k));
    [logs.rollNose_deg(k), logs.swing_deg(k)] = roll_about_nose(q, qRef);

    % Nose direction is Xb in display ENU; elevation remains defined at vertical.
    nose = enu_from_body(q) * [1; 0; 0];
    logs.noseElev_deg(k) = asind(min(1, max(-1, nose(3))));
    logs.noseAzim_deg(k) = atan2d(nose(1), nose(2));   % North reference, positive toward east
end

logs.rollNose_deg = rad2deg(unwrap(deg2rad(logs.rollNose_deg)));
end

function out = merge_structs(defaults, data)
out   = defaults;
names = fieldnames(data);
for k = 1:numel(names)
    out.(names{k}) = data.(names{k});
end
end

function tf = has_log(logs, name)
tf = isfield(logs, name) && ~isempty(logs.(name));
end

function value = log_value(logs, name, k)
if has_log(logs, name)
    value = logs.(name)(k);
else
    value = nan;
end
end


%% ======================================================================
%  Frames and attitude
%  ======================================================================
function R = enu_from_uen()
%ENU_FROM_UEN  Reorder model UEN to display ENU: (u, e, n) -> (e, n, u).
R = [0 1 0; 0 0 1; 1 0 0];
end

function R = enu_from_body(q)
%ENU_FROM_BODY  Rotation matrix mapping body-axis vectors to display ENU.
%   quat_to_rotm(q) returns the active rotation matrix "UEN <- body". The
%   6DOF (Quaternion) output satisfies quat2dcm(q) = DCM_body_from_UEN; no extra transpose is needed here.
%   Body axes (x forward / y left / z up) coincide with the local mesh axes
%   in the UEN convention, so the mesh needs no additional transformation.
R = enu_from_uen() * quat_to_rotm(q);
end

function R = quat_to_rotm(q)
q = normalize_quaternion(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1 - 2*(y^2 + z^2), 2*(x*y - z*w),     2*(x*z + y*w);
     2*(x*y + z*w),     1 - 2*(x^2 + z^2), 2*(y*z - x*w);
     2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x^2 + y^2)];
end

function [rollDeg, swingDeg] = roll_about_nose(q, qRef)
%ROLL_ABOUT_NOSE  Use swing-twist decomposition for roll about the reference nose axis.
%   Unlike 321 Euler phi, this is unique and nonsingular unless the nose
%   is 180 deg from the reference. ROLLDEG is twist; SWINGDEG is nose deviation.
%   (For the same calculation in a Simulink MATLAB Function block,
%    see the code-generation-compatible quat_roll_about_nose.m.)
qErr = quaternion_product(quaternion_conjugate(qRef), q);
if qErr(1) < 0
    qErr = -qErr;   % Choose the shorter rotation representation.
end
rollDeg  = 2 * atan2d(qErr(2), qErr(1));
swingDeg = 2 * atan2d(hypot(qErr(3), qErr(4)), hypot(qErr(1), qErr(2)));
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


%% ======================================================================
%  Figure / layout construction
%  ======================================================================
function ui = build_ui(logs, logsFull, opts)
ui.fig = create_figure();
worldLimits = world_axis_limits(logs.position_m, opts.BodyLength);
worldScale  = resolve_world_scale(worldLimits, opts);

[ui.axWorld, ui.axInfo, ui.axFrame, layout, trendSpan] = build_layout(ui.fig, opts);

ui.world = build_world_view(ui.axWorld, logs, opts, worldLimits, worldScale);
if isempty(ui.axFrame)
    ui.info  = struct([]);
    ui.frame = struct([]);
else
    ui.info  = build_info_panel(ui.axInfo);
    ui.frame = build_frame_panel(ui.axFrame, logs, opts);
end

if opts.ShowTrends
    ui.trends = build_trend_panel(layout, logsFull, trendSpan);
else
    ui.trends = struct([]);
end
end

function fig = create_figure()
screen = get(groot, "ScreenSize");
width  = min(1500, screen(3) - 80);
height = min(900,  screen(4) - 140);
fig = figure( ...
    "Name", "Rocket Flight Animation", ...
    "Color", "w", ...
    "NumberTitle", "off", ...
    "Position", [screen(1) + 40, screen(2) + 70, width, height]);

end


function [axWorld, axInfo, axFrame, layout, trendSpan] = build_layout(fig, opts)
%BUILD_LAYOUT  World view on the left; readout, frames, and histories on the right.
nTrendRows = 3 * double(opts.ShowTrends);
nRows      = 3 + nTrendRows;
nCols      = 5;
nWorldCols = 2;
nInfoCols  = 1;
nRightCols = nCols - nWorldCols;

if opts.Layout == "worldonly"
    axInfo  = gobjects(0);
    axFrame = gobjects(0);
    if opts.ShowTrends
        layout    = tiledlayout(fig, nTrendRows, nCols, "TileSpacing", "compact", "Padding", "compact");
        axWorld   = nexttile(layout, [nTrendRows, 3]);
        trendSpan = [1 2];
    else
        layout    = tiledlayout(fig, 1, 1, "TileSpacing", "compact", "Padding", "compact");
        axWorld   = nexttile(layout);
        trendSpan = [1 1];
    end
    return
end

layout    = tiledlayout(fig, nRows, nCols, "TileSpacing", "compact", "Padding", "compact");
axWorld   = nexttile(layout, [nRows, nWorldCols]);
axInfo    = nexttile(layout, [3, nInfoCols]);
axFrame   = nexttile(layout, [3, nRightCols - nInfoCols]);
trendSpan = [1 nRightCols];
end


% --- World view ---------------------------------------------------------
function world = build_world_view(ax, logs, opts, limits, worldScale)
prepare_3d_axis(ax, limits);
view(ax, opts.WorldView(1), opts.WorldView(2));
title(ax, "World view (UEN inertial frame)");

triadLength = triad_length(limits, [0; 0; 0]);

world.ground = create_ground_plane(ax, mean(limits(1:2, :), 2), ...
    0.5 * max(diff(limits(1:2, :), 1, 2)));
world.enuTriad = create_triad(ax, triadLength, ["E", "N", "U"], reference_triad_colors(), 2.4);
update_triad(world.enuTriad, eye(3), [0; 0; 0], triadLength);

world.pathFull = plot3(ax, logs.position_m(1, :), logs.position_m(2, :), logs.position_m(3, :), ...
    "Color", [0.80 0.82 0.86], "LineWidth", 0.8);
world.trail    = plot3(ax, nan, nan, nan, "Color", [0.15 0.45 0.80], "LineWidth", 1.6);
world.shadow   = plot3(ax, nan, nan, nan, "Color", [0.55 0.55 0.55], "LineWidth", 0.8);
world.dropLine = plot3(ax, nan, nan, nan, ":", "Color", [0.45 0.45 0.45], "LineWidth", 0.8);
world.marker   = plot3(ax, nan, nan, nan, ".", "MarkerSize", 14, "Color", [0.85 0.20 0.20]);
set([world.trail, world.shadow], "Visible", on_off(opts.Trail));

% Apply the display scale to geometry here, never again during attitude updates.
% Applying it in both places would square the scale factor.
world.rocket    = create_rocket_graphics(ax, worldScale * opts.BodyLength, opts.CgFraction, false);
world.chute     = create_chute_graphics(ax, opts.ChuteDiameter, opts.ChuteRiser, worldScale);
world.noseOffset = worldScale * opts.BodyLength * (1 - opts.CgFraction);
world.showTrail = opts.Trail;

% The camera is fixed, so create lighting once instead of recomputing each frame.
light(ax, "Style", "infinite", "Position", [1 -1 2]);
end

% --- Coordinate frames (separate inertial and body axes) -----------------
function frame = build_frame_panel(ax, logs, opts)
hold(ax, "on");
grid(ax, "off");
axis(ax, "off");
daspect(ax, [1 1 1]);   % Avoid vis3d, which can cause clipping as in the world view.
set(ax, "XLim", [-1.5 1.5], "YLim", [-1.5 1.5], "ZLim", [-1.5 1.5], ...
    "Projection", "orthographic");
view(ax, opts.FrameView(1), opts.FrameView(2));
title(ax, "Frames (inertial UEN / body Xb Yb Zb)");

% Horizontal reference ring for visualizing tilt.
ring = linspace(0, 2 * pi, 80);
plot3(ax, cos(ring), sin(ring), zeros(size(ring)), "Color", [0.80 0.80 0.80], "LineWidth", 0.8);

% Keep faint initial Yb / Zb axes visible as a reference for accumulated roll.
R0 = enu_from_body(logs.quaternionBI(:, 1));
for k = 2:3
    tip = 1.05 * R0(:, k);
    plot3(ax, [0 tip(1)], [0 tip(2)], [0 tip(3)], "--", ...
        "Color", [0.72 0.74 0.78], "LineWidth", 1.2);
end

% Thin gray inertial axes and thick colored body axes are easy to distinguish.
frame.enuTriad = create_triad(ax, 1.25, ["E", "N", "U"], world_triad_colors(), 1.2);
update_triad(frame.enuTriad, eye(3), [0; 0; 0], 1.25);
frame.bodyTriad = create_triad(ax, 1.05, ["Xb", "Yb", "Zb"], body_triad_colors(), 3.0);

frame.rocket = create_rocket_graphics(ax, 1.1, 0.5, true);
frame.chute  = create_chute_graphics(ax, 0.9, 0.9, 1);
frame.noseOffset = 0.55;
light(ax, "Style", "infinite", "Position", [1 -1 1.5]);
end

% --- Numeric readout (separate tile from the frame panel) ----------------
function info = build_info_panel(ax)
axis(ax, "off");
title(ax, "Flight state");
info.readout = text(ax, 0, 0, "", ...
    "Units", "normalized", ...
    "Position", [0.04 0.96], ...
    "VerticalAlignment", "top", ...
    "HorizontalAlignment", "left", ...
    "FontName", "Consolas", ...
    "FontSize", 9);
end

% --- Time-history panel -------------------------------------------------
function trends = build_trend_panel(layout, logs, span)
spec = trend_specification();
trends = struct("ax", {}, "cursor", {});

for k = 1:numel(spec)
    ax = nexttile(layout, span);
    hold(ax, "on");
    grid(ax, "on");
    box(ax, "on");
    ylabel(ax, spec(k).label);

    names = string.empty;
    names = plot_trend_line(ax, logs, spec(k).field,    names, spec(k).legend(1), "-",  [0.10 0.35 0.70], 1.4);
    names = plot_trend_line(ax, logs, spec(k).cmdField, names, spec(k).legend(2), "--", [0.85 0.35 0.10], 1.0);

    if isempty(names)
        text(ax, 0.5, 0.5, "no data", "Units", "normalized", ...
            "HorizontalAlignment", "center", "Color", [0.6 0.6 0.6]);
    else
        legend(ax, names, "Location", "northeast", "Box", "off", "FontSize", 8, ...
            "AutoUpdate", "off");
    end

    % Updating xline (ConstantLine) redraws the entire axes and updates the legend.
    % A two-point line looks the same and only requires changing XData.
    xlim(ax, [logs.t(1) logs.t(end)]);
    ylim(ax, ylim(ax));
    set(ax, "XLimMode", "manual", "YLimMode", "manual");
    cursor = plot(ax, [logs.t(1) logs.t(1)], ylim(ax), ...
        "Color", [0.80 0.20 0.20], "LineWidth", 1.0);
    cursor.Annotation.LegendInformation.IconDisplayStyle = "off";

    if k == numel(spec)
        xlabel(ax, "Time [s]");
    else
        set(ax, "XTickLabel", []);
    end

    trends(k).ax     = ax;
    trends(k).cursor = cursor;
end
end

function names = plot_trend_line(ax, logs, field, names, label, style, color, width)
if strlength(field) == 0 || ~has_log(logs, field)
    return
end
plot(ax, logs.t, logs.(field), style, "LineWidth", width, "Color", color);
names(end + 1) = label;
end

function spec = trend_specification()
spec = struct( ...
    "field",    {"altitude_m", "speed_mps", "qbar"}, ...
    "cmdField", {"",           "",          ""}, ...
    "label",    {"altitude [m]", "speed [m/s]", "qbar [Pa]"}, ...
    "legend",   {["altitude", ""], ["speed", ""], ["qbar", ""]});
end


%% ======================================================================
%  Axis helpers
%  ======================================================================
function prepare_3d_axis(ax, limits)
hold(ax, "on");
grid(ax, "on");
box(ax, "off");
xlabel(ax, "East [m]");
ylabel(ax, "North [m]");
zlabel(ax, "Up [m]");
set(ax, ...
    "XLim", limits(1, :), ...
    "YLim", limits(2, :), ...
    "ZLim", limits(3, :), ...
    "Projection", "orthographic", ...
    "XLimMode", "manual", "YLimMode", "manual", "ZLimMode", "manual");
% Avoid axis equal, which conflicts with manual XLim. A manual daspect
% disables stretch-to-fill while preserving the aspect ratio.
% Avoid axis vis3d: it fixes CameraViewAngleMode to manual, preventing
% automatic shrinking when an oblique projection extends beyond the axes.
daspect(ax, [1 1 1]);
lighting(ax, "flat");
end

function len = triad_length(limits, origin)
%TRIAD_LENGTH  Choose a triad length that fits within the axes limits.
%   Axes extend only in the positive direction from the origin; use the
%   smallest upper-limit margin and allow for labels at 1.14 times the length.
labelMargin = 1.14;
room = min(limits(:, 2) - origin(:));
len  = 0.70 * max(room, 0) / labelMargin;
end

function limits = world_axis_limits(position, bodyLength)
low  = min(position, [], 2);
high = max(position, [], 2);
low(3) = min(low(3), 0);

verticalSpan  = max(high(3) - low(3), 8 * bodyLength);
minHorizontal = max(0.30 * verticalSpan, 10 * bodyLength);
for k = 1:2
    if high(k) - low(k) < minHorizontal
        center  = 0.5 * (low(k) + high(k));
        low(k)  = center - 0.5 * minHorizontal;
        high(k) = center + 0.5 * minHorizontal;
    end
end

pad = 0.05 * verticalSpan;
limits = [low(1) - pad, high(1) + pad;
          low(2) - pad, high(2) + pad;
          low(3),       high(3) + pad];
end

function scale = resolve_world_scale(limits, opts)
if ~isempty(opts.WorldScale)
    scale = opts.WorldScale;
    return
end
% Choose a rounded scale so the vehicle is about 5% of the world-view height.
scale = max(1, round(0.05 * diff(limits(3, :)) / opts.BodyLength));
end

function state = on_off(flag)
if flag
    state = "on";
else
    state = "off";
end
end

function h = create_ground_plane(ax, centerEN, halfSpan)
halfSpan = max(halfSpan, 1);
[e, n] = meshgrid(centerEN(1) + halfSpan * [-1 1], centerEN(2) + halfSpan * [-1 1]);
h = surface(ax, e, n, zeros(2), ...
    ... Avoid FaceAlpha below 1: it enables depth sorting and can multiply
    ... the rendering cost per frame. Use a lighter color instead.
    "FaceColor", [0.95 0.96 0.94], ...
    "EdgeColor", [0.82 0.83 0.80], ...
    "FaceLighting", "none");
end


%% ======================================================================
%  Reference-frame triads
%  ======================================================================
function colors = world_triad_colors()
colors = repmat([0.45 0.45 0.48], 3, 1);
end

function colors = reference_triad_colors()
colors = repmat([0.10 0.35 0.60], 3, 1);
end

function colors = body_triad_colors()
colors = [0.85 0.16 0.16;   % Xb: nose direction
          0.10 0.62 0.28;   % Yb: east at zero attitude
          0.15 0.35 0.90];  % Zb: north at zero attitude
end

function triad = create_triad(ax, len, labels, colors, lineWidth)
triad.line = gobjects(1, 3);
triad.text = gobjects(1, 3);
for k = 1:3
    triad.line(k) = plot3(ax, nan, nan, nan, "Color", colors(k, :), "LineWidth", lineWidth);
    triad.text(k) = text(ax, nan, nan, nan, labels(k), ...
        "Color", colors(k, :), "FontWeight", "bold", "FontSize", 9, ...
        "HorizontalAlignment", "center");
    triad.line(k).Annotation.LegendInformation.IconDisplayStyle = "off";
end
triad.length = len;
end

function update_triad(triad, R, origin, len)
% Each column of R is an axis direction in display coordinates.
for k = 1:3
    tip = origin + len * R(:, k);
    set(triad.line(k), ...
        "XData", [origin(1) tip(1)], ...
        "YData", [origin(2) tip(2)], ...
        "ZData", [origin(3) tip(3)]);
    set(triad.text(k), "Position", (origin + 1.14 * len * R(:, k)).');
end
end


%% ======================================================================
%  Rocket graphics
%  ======================================================================
function rocket = create_rocket_graphics(ax, bodyLength, cgFraction, showTailRing)
geom = rocket_geometry(bodyLength, cgFraction);

rocket.surfaces = [ ...
    surf(ax, geom.body.X, geom.body.Y, geom.body.Z, ...
        "FaceColor", [0.90 0.91 0.94], "EdgeColor", "none", ...
        "FaceLighting", "flat", "AmbientStrength", 0.55, "UserData", geom.body), ...
    surf(ax, geom.nose.X, geom.nose.Y, geom.nose.Z, ...
        "FaceColor", [0.78 0.18 0.16], "EdgeColor", "none", ...
        "FaceLighting", "flat", "AmbientStrength", 0.55, "UserData", geom.nose)];

% Roll-reference stripe; keep fins the same color to reserve colors for axes.
rocket.lines = plot3(ax, geom.stripe(:, 1), geom.stripe(:, 2), geom.stripe(:, 3), ...
    "LineWidth", 2.0, "Color", [0.95 0.60 0.10], "UserData", geom.stripe);

rocket.patches = gobjects(1, 4);
for k = 1:4
    rocket.patches(k) = patch(ax, ...
        "XData", geom.fin{k}(:, 1), "YData", geom.fin{k}(:, 2), "ZData", geom.fin{k}(:, 3), ...
        "FaceColor", [0.66 0.69 0.74], "EdgeColor", [0.35 0.36 0.40], ...
        "UserData", geom.fin{k});
end

if showTailRing
    rocket.lines(end + 1) = plot3(ax, geom.tail(:, 1), geom.tail(:, 2), geom.tail(:, 3), ...
        "LineWidth", 1.2, "Color", [0.25 0.25 0.28], "UserData", geom.tail);
end

for h = [rocket.surfaces, rocket.lines, rocket.patches]
    h.Annotation.LegendInformation.IconDisplayStyle = "off";
end
end

function chute = create_chute_graphics(ax, diameter, riserLength, scale)
%CREATE_CHUTE_GRAPHICS  Create a hemispherical canopy and parachute risers.
%   Place the canopy riserLength from the attachment, opposite the relative
%   air velocity. Scale with inflation fraction to show deployment in progress.
nu = 12; nv = 6;
theta = linspace(0, 2*pi, nu);
phi   = linspace(0, pi/2, nv).';
radius = 0.5 * diameter * scale;
chute.canopyLocal.X = radius * (sin(phi) * cos(theta));
chute.canopyLocal.Y = radius * (sin(phi) * sin(theta));
chute.canopyLocal.Z = radius * repmat(cos(phi), 1, nu);
chute.riserLocal = radius;
chute.scale = scale;
chute.riserLength = riserLength * scale;

chute.canopy = surf(ax, nan(nv, nu), nan(nv, nu), nan(nv, nu), ...
    "FaceColor", [0.90 0.35 0.20], "EdgeColor", [0.55 0.20 0.10], ...
    "FaceLighting", "flat", "AmbientStrength", 0.6, "Visible", "off");
chute.lines = plot3(ax, nan, nan, nan, "Color", [0.35 0.35 0.38], ...
    "LineWidth", 0.8, "Visible", "off");
chute.canopy.Annotation.LegendInformation.IconDisplayStyle = "off";
chute.lines.Annotation.LegendInformation.IconDisplayStyle = "off";
end


function update_chute(chute, attachPoint, upDir, openFraction)
%UPDATE_CHUTE  Place the canopy above the attachment, opposite the relative air velocity.
%   Hide at openFraction = 0; fully open at 1; scale radius and distance in between.
if openFraction < 0.02
    set(chute.canopy, "Visible", "off");
    set(chute.lines,  "Visible", "off");
    return
end

scale  = max(openFraction, 0.05);
center = attachPoint(:) + chute.riserLength * scale * upDir(:);

% Rotate the local canopy z-axis into upDir.
R = frame_from_axis(upDir(:));
[x, y, z] = transform_mesh(scale * chute.canopyLocal.X, ...
    scale * chute.canopyLocal.Y, scale * chute.canopyLocal.Z, R, center);
set(chute.canopy, "XData", x, "YData", y, "ZData", z, "Visible", "on");

% Risers run from the attachment to points along the canopy rim.
rim = [x(1, :); y(1, :); z(1, :)];
pick = 1:3:size(rim, 2);
lx = []; ly = []; lz = [];
for k = pick
    lx = [lx, attachPoint(1), rim(1, k), nan]; %#ok<AGROW>
    ly = [ly, attachPoint(2), rim(2, k), nan]; %#ok<AGROW>
    lz = [lz, attachPoint(3), rim(3, k), nan]; %#ok<AGROW>
end
set(chute.lines, "XData", lx, "YData", ly, "ZData", lz, "Visible", "on");
end


function R = frame_from_axis(zAxis)
%FRAME_FROM_AXIS  Construct an orthonormal basis with the given direction as z-axis.
zAxis = zAxis / max(norm(zAxis), eps);
ref = [1; 0; 0];
if abs(dot(ref, zAxis)) > 0.9
    ref = [0; 1; 0];
end
xAxis = cross(ref, zAxis); xAxis = xAxis / max(norm(xAxis), eps);
yAxis = cross(zAxis, xAxis);
R = [xAxis, yAxis, zAxis];
end


function geom = rocket_geometry(bodyLength, cgFraction)
% Local mesh axes: x forward / y left / z up, matching body axes in UEN.
% Set the origin at cgFraction, the reference point of the position log.
radius  = 0.05 * bodyLength;
bodyEnd = 0.82 * bodyLength;
xShift  = -cgFraction * bodyLength;
theta   = linspace(0, 2 * pi, 12);
unitRow = ones(size(theta));

geom.body.X = [zeros(size(theta)); bodyEnd * unitRow] + xShift;
geom.body.Y = radius * [cos(theta); cos(theta)];
geom.body.Z = radius * [sin(theta); sin(theta)];

geom.nose.X = [bodyEnd * unitRow; bodyLength * unitRow] + xShift;
geom.nose.Y = [radius * cos(theta); zeros(size(theta))];
geom.nose.Z = [radius * sin(theta); zeros(size(theta))];

tailTheta = linspace(0, 2 * pi, 16).';
geom.tail = [xShift * ones(size(tailTheta)), radius * cos(tailTheta), radius * sin(tailTheta)];

xStripe = linspace(0.12 * bodyLength, 0.90 * bodyEnd, 20).' + xShift;
geom.stripe = [xStripe, 0.90 * radius * ones(size(xStripe)), 0.30 * radius * ones(size(xStripe))];

rootLE = 0.04 * bodyLength + xShift;
rootTE = 0.18 * bodyLength + xShift;
tipLE  = 0.08 * bodyLength + xShift;
tipTE  = 0.15 * bodyLength + xShift;
span   = 0.16 * bodyLength;

geom.fin = { ...
    [rootLE  radius 0; rootTE  radius 0; tipTE  span 0; tipLE  span 0], ...
    [rootLE -radius 0; rootTE -radius 0; tipTE -span 0; tipLE -span 0], ...
    [rootLE 0  radius; rootTE 0  radius; tipTE 0  span; tipLE 0  span], ...
    [rootLE 0 -radius; rootTE 0 -radius; tipTE 0 -span; tipLE 0 -span]};
end

function update_rocket_pose(rocket, position, R)
for h = rocket.surfaces
    geom = h.UserData;
    [x, y, z] = transform_mesh(geom.X, geom.Y, geom.Z, R, position);
    set(h, "XData", x, "YData", y, "ZData", z);
end
for h = [rocket.lines, rocket.patches]
    pts = h.UserData * R.' + position.';
    set(h, "XData", pts(:, 1), "YData", pts(:, 2), "ZData", pts(:, 3));
end
end

function [x, y, z] = transform_mesh(xLocal, yLocal, zLocal, R, position)
pts = [xLocal(:), yLocal(:), zLocal(:)] * R.' + position.';
x = reshape(pts(:, 1), size(xLocal));
y = reshape(pts(:, 2), size(yLocal));
z = reshape(pts(:, 3), size(zLocal));
end


%% ======================================================================
%  Animation loop
%  ======================================================================
function run_animation(ui, logs, opts)
% In Live Script figures, drawnow inside the loop may leave only a final
% still frame. Register animation start and end to include the animation
% with playback controls in the Live Script output.
video = open_video(opts, logs);
cleanup = onCleanup(@() close_video(video));
frameSize = [];

tic;
for k = 1:numel(logs.t)
    position = logs.position_m(:, k);
    R = enu_from_body(logs.quaternionBI(:, k));

    chuteOpen = log_value(logs, "chuteOpen", k);
    if ~isfinite(chuteOpen)
        chuteOpen = 0;
    end
    update_world_view(ui.world, logs, k, position, R, chuteOpen);
    if ~isempty(ui.frame)
        update_frame_panel(ui.frame, R, chuteOpen);
        set(ui.info.readout, "String", format_readout(logs, k));
    end
    update_trends(ui.trends, logs.t(k));

    % Do not use limitrate: it skips rendering and drops frames.
    % Do not update titles each frame, which triggers axes layout calculations.
    % Time is already shown in the flight-state readout.
    drawnow;

    if opts.SaveVideo
        [video, frameSize] = write_frame(video, ui.fig, frameSize);
    else
        pause(max(0, (k - 1) / opts.FrameRate - toc));
    end
end
end

function update_world_view(world, logs, k, position, R, chuteOpen)
update_rocket_pose(world.rocket, position, R);
if world.showTrail
    set(world.trail, ...
        "XData", logs.position_m(1, 1:k), ...
        "YData", logs.position_m(2, 1:k), ...
        "ZData", logs.position_m(3, 1:k));
    set(world.shadow, ...
        "XData", logs.position_m(1, 1:k), ...
        "YData", logs.position_m(2, 1:k), ...
        "ZData", zeros(1, k));
end
set(world.dropLine, ...
    "XData", [position(1) position(1)], ...
    "YData", [position(2) position(2)], ...
    "ZData", [0 position(3)]);
set(world.marker, "XData", position(1), "YData", position(2), "ZData", position(3));

% Extend the canopy forward from the nose-side attachment (upward during descent).
noseDir = R * [1; 0; 0];
update_chute(world.chute, position + world.noseOffset * noseDir, noseDir, chuteOpen);
end

function update_frame_panel(frame, R, chuteOpen)
update_triad(frame.bodyTriad, R, [0; 0; 0], 1.05);
update_rocket_pose(frame.rocket, [0; 0; 0], R);
noseDir = R * [1; 0; 0];
update_chute(frame.chute, frame.noseOffset * noseDir, noseDir, chuteOpen);
end

function update_trends(trends, tNow)
for k = 1:numel(trends)
    set(trends(k).cursor, "XData", [tNow tNow]);
end
end

function txt = format_readout(logs, k)
spec = readout_specification();
txt  = strings(numel(spec), 1);
for n = 1:numel(spec)
    lineFormat = "%-6s" + spec(n).format + " %s";
    txt(n) = sprintf(lineFormat, spec(n).label, log_value(logs, spec(n).field, k), spec(n).unit);
end
end

function spec = readout_specification()
% All quantities come from quaternions: roll is nose-axis twist; swing is nose deviation.
spec = struct( ...
    "field",  {"t",     "altitude_m", "east_m", "north_m", "range_m", "speed_mps", "noseElev_deg", "noseAzim_deg", "rollNose_deg"}, ...
    "label",  {"t",     "alt",        "east",   "north",   "range",   "V",         "elev",         "azim",         "roll"}, ...
    "unit",   {"s",     "m",          "m",      "m",       "m",       "m/s",       "deg",          "deg",          "deg"}, ...
    "format", {"%7.2f", "%7.1f",      "%7.1f",  "%7.1f",   "%7.1f",   "%7.1f",     "%7.1f",        "%7.1f",        "%7.1f"});
end


%% ======================================================================
%  Video output
%  ======================================================================
function video = open_video(opts, ~)
video = [];
if ~opts.SaveVideo
    return
end
video = VideoWriter(char(opts.VideoFile), "MPEG-4");
video.FrameRate = min(60, max(1, round(opts.FrameRate)));
open(video);
end

function [video, frameSize] = write_frame(video, fig, frameSize)
frame = getframe(fig);
if isempty(frameSize)
    frameSize = size(frame.cdata, [1 2]);
elseif ~isequal(size(frame.cdata, [1 2]), frameSize)
    frame.cdata = imresize(frame.cdata, frameSize);
end
writeVideo(video, frame);
end

function close_video(video)
if ~isempty(video) && isobject(video)
    close(video);
end
end
