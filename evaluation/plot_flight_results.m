function summary = plot_flight_results(simOut, figureName)
%PLOT_FLIGHT_RESULTS Plot trajectory, ground track, altitude, and speed.

arguments
    simOut
    figureName (1,1) string = "6DOF Flight Result"
end

position = get_logged_matrix(simOut, "positionUEN");
velocity = get_logged_matrix(simOut, "velocityBody");
if isempty(position)
    error("plot_flight_results:MissingPosition", ...
        "positionUEN was not found in the logs.");
end

summary = workshop_flight_summary(simOut);
east = position.data(:, 2);
north = position.data(:, 3);
up = position.data(:, 1);

fig = figure("Name", figureName, "Color", "w", ...
    "Position", [100 100 1100 760]);
tiledlayout(fig, 2, 2, "TileSpacing", "compact", "Padding", "compact");

ax = nexttile;
plot3(ax, east, north, up, "LineWidth", 1.5);
hold(ax, "on");
plot3(ax, summary.LandingEast_m, summary.LandingNorth_m, 0, ...
    "rx", "MarkerSize", 10, "LineWidth", 2);
grid(ax, "on");
xlabel(ax, "East [m]");
ylabel(ax, "North [m]");
zlabel(ax, "Up [m]");
title(ax, "Flight trajectory");
view(ax, 38, 24);

ax = nexttile;
plot(ax, east, north, "LineWidth", 1.5);
hold(ax, "on");
plot(ax, 0, 0, "ko", "MarkerFaceColor", "k", "DisplayName", "Launch");
plot(ax, summary.LandingEast_m, summary.LandingNorth_m, ...
    "rx", "MarkerSize", 10, "LineWidth", 2, "DisplayName", "Landing");
axis(ax, "equal");
grid(ax, "on");
xlabel(ax, "East [m]");
ylabel(ax, "North [m]");
title(ax, sprintf("Ground track: range %.0f m", summary.LandingRange_m));
legend(ax, "Location", "best");

ax = nexttile;
plot(ax, position.time, up, "LineWidth", 1.5);
grid(ax, "on");
xlabel(ax, "Time [s]");
ylabel(ax, "Altitude [m]");
title(ax, sprintf("Apogee %.0f m", summary.Apogee_m));

ax = nexttile;
if isempty(velocity)
    text(ax, 0.5, 0.5, "velocityBody is not logged", ...
        "Units", "normalized", "HorizontalAlignment", "center");
else
    speed = vecnorm(velocity.data, 2, 2);
    plot(ax, velocity.time, speed, "LineWidth", 1.5);
end
grid(ax, "on");
xlabel(ax, "Time [s]");
ylabel(ax, "Speed [m/s]");
title(ax, sprintf("Maximum speed %.0f m/s", summary.MaxSpeed_mps));

sgtitle(fig, figureName);
end

function signal = get_logged_matrix(simOut, signalName)
signal = [];
try
    values = simOut.logsout.get(char(signalName)).Values;
catch
    return
end
data = squeeze(values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(values.Time)
    data = data.';
end
signal = struct("time", values.Time(:), "data", data);
end
