function summary = workshop_flight_summary(simOut)
%WORKSHOP_FLIGHT_SUMMARY Calculate key flight metrics from simulation logs.
%
%   Position is interpreted as UEN: Up, East, North. Landing coordinates
%   are linearly interpolated at the final downward crossing of ground.

arguments
    simOut
end

nRun = numel(simOut);
runNumber = (1:nRun).';
flightTime = nan(nRun, 1);
apogee = nan(nRun, 1);
maxSpeed = nan(nRun, 1);
landingEast = nan(nRun, 1);
landingNorth = nan(nRun, 1);
landingRange = nan(nRun, 1);

for k = 1:nRun
    position = get_logged_matrix(simOut(k), "positionUEN");
    velocity = get_logged_matrix(simOut(k), "velocityBody");
    if isempty(position)
        continue
    end

    [landingTime, landingPosition] = interpolate_landing( ...
        position.time, position.data);
    flightTime(k) = landingTime;
    apogee(k) = max(position.data(:, 1));
    landingEast(k) = landingPosition(2);
    landingNorth(k) = landingPosition(3);
    landingRange(k) = hypot(landingEast(k), landingNorth(k));

    if ~isempty(velocity)
        maxSpeed(k) = max(vecnorm(velocity.data, 2, 2));
    end
end

summary = table(runNumber, flightTime, apogee, maxSpeed, ...
    landingEast, landingNorth, landingRange, ...
    'VariableNames', {'Run', 'FlightTime_s', 'Apogee_m', 'MaxSpeed_mps', ...
    'LandingEast_m', 'LandingNorth_m', 'LandingRange_m'});
end

function [landingTime, landingPosition] = interpolate_landing(time, position)
altitude = position(:, 1);
crossing = find(altitude(1:end-1) > 0 & altitude(2:end) <= 0, 1, "last");
if isempty(crossing)
    landingTime = time(end);
    landingPosition = position(end, :);
    return
end

k = crossing;
fraction = altitude(k) / (altitude(k) - altitude(k + 1));
landingTime = time(k) + fraction * (time(k + 1) - time(k));
landingPosition = position(k, :) + ...
    fraction * (position(k + 1, :) - position(k, :));
landingPosition(1) = 0;
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
