function plot_landing_dispersion(results, figureName)
%PLOT_LANDING_DISPERSION Plot Monte Carlo landing points and percentiles.

arguments
    results table
    figureName (1,1) string = "Monte Carlo Landing Dispersion"
end

required = ["LandingEast_m", "LandingNorth_m"];
missing = setdiff(required, string(results.Properties.VariableNames));
if ~isempty(missing)
    error("plot_landing_dispersion:MissingColumns", ...
        "The results table requires these columns: %s", strjoin(missing, ", "));
end

east = results.LandingEast_m;
north = results.LandingNorth_m;
valid = isfinite(east) & isfinite(north);
east = east(valid);
north = north(valid);
if isempty(east)
    error("plot_landing_dispersion:NoValidResults", ...
        "No valid landing locations are available.");
end

fig = figure("Name", figureName, "Color", "w", ...
    "Position", [180 140 760 680]);
ax = axes(fig);
hold(ax, "on");
grid(ax, "on");

if ismember("WindSpeed", string(results.Properties.VariableNames))
    colorValue = results.WindSpeed(valid);
    scatter(ax, east, north, 32, colorValue, "filled", ...
        "MarkerFaceAlpha", 0.70, "DisplayName", "Landing points");
    cb = colorbar(ax);
    cb.Label.String = "Wind speed [m/s]";
else
    scatter(ax, east, north, 32, "filled", ...
        "MarkerFaceAlpha", 0.70, "DisplayName", "Landing points");
end

center = [mean(east), mean(north)];
plot(ax, 0, 0, "kp", "MarkerFaceColor", "y", ...
    "MarkerSize", 12, "DisplayName", "Launch");
plot(ax, center(1), center(2), "kx", ...
    "MarkerSize", 11, "LineWidth", 2, "DisplayName", "Mean landing");

if numel(east) >= 3
    draw_probability_ellipse(ax, east, north, 0.50, "--", "50% ellipse");
    draw_probability_ellipse(ax, east, north, 0.90, "-", "90% ellipse");
end

axis(ax, "equal");
xlabel(ax, "East [m]");
ylabel(ax, "North [m]");
title(ax, figureName);
legend(ax, "Location", "best");
end

function draw_probability_ellipse(ax, east, north, probability, style, name)
covariance = cov([east, north]);
[vectors, values] = eig(covariance);
radii = sqrt(max(diag(values), 0)) * sqrt(-2 * log(1 - probability));
angle = linspace(0, 2 * pi, 181);
ellipse = mean([east, north], 1).' + ...
    vectors * diag(radii) * [cos(angle); sin(angle)];
plot(ax, ellipse(1, :), ellipse(2, :), style, ...
    "Color", [0.15 0.15 0.15], "LineWidth", 1.4, ...
    "DisplayName", name);
end
