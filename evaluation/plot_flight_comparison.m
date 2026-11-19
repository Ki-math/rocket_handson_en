function summaries = plot_flight_comparison(simOut, labels, figureName)
%PLOT_FLIGHT_COMPARISON Compare trajectories from several flight cases.

arguments
    simOut
    labels string = "Case " + (1:numel(simOut))
    figureName (1,1) string = "Flight Condition Comparison"
end

labels = labels(:);
if numel(labels) ~= numel(simOut)
    error("plot_flight_comparison:LabelCount", ...
        "The number of labels must match the number of simulations.");
end

summaries = workshop_flight_summary(simOut);
colors = lines(numel(simOut));
fig = figure("Name", figureName, "Color", "w", ...
    "Position", [120 120 1100 520]);
tiledlayout(fig, 1, 2, "TileSpacing", "compact", "Padding", "compact");

groundAxis = nexttile;
hold(groundAxis, "on");
grid(groundAxis, "on");
axis(groundAxis, "equal");
xlabel(groundAxis, "East [m]");
ylabel(groundAxis, "North [m]");
title(groundAxis, "Ground track");

altitudeAxis = nexttile;
hold(altitudeAxis, "on");
grid(altitudeAxis, "on");
xlabel(altitudeAxis, "Time [s]");
ylabel(altitudeAxis, "Altitude [m]");
title(altitudeAxis, "Altitude history");

for k = 1:numel(simOut)
    position = get_position(simOut(k));
    plot(groundAxis, position(:, 2), position(:, 3), ...
        "Color", colors(k, :), "LineWidth", 1.4, ...
        "DisplayName", labels(k));
    plot(groundAxis, summaries.LandingEast_m(k), ...
        summaries.LandingNorth_m(k), "x", ...
        "Color", colors(k, :), "MarkerSize", 8, "LineWidth", 1.5, ...
        "HandleVisibility", "off");
    plot(altitudeAxis, position(:, 4), position(:, 1), ...
        "Color", colors(k, :), "LineWidth", 1.4, ...
        "DisplayName", labels(k));
end
plot(groundAxis, 0, 0, "ko", "MarkerFaceColor", "k", ...
    "DisplayName", "Launch");
legend(groundAxis, "Location", "best");
legend(altitudeAxis, "Location", "best");
sgtitle(fig, figureName);
end

function position = get_position(simOut)
try
    values = simOut.logsout.get("positionUEN").Values;
catch
    error("plot_flight_comparison:MissingPosition", ...
        "positionUEN was not found in the logs.");
end
data = squeeze(values.Data);
if size(data, 1) ~= numel(values.Time)
    data = data.';
end
position = [data, values.Time(:)];
end
