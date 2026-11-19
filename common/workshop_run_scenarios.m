function simOut = workshop_run_scenarios(scenarios, options)
%WORKSHOP_RUN_SCENARIOS Run flight cases defined by a table.
%
%   Required columns:
%     LaunchTiltDeg, LaunchAzimuthDeg, WindSpeed, WindFromDeg
%
%   LaunchTiltDeg is measured from vertical. Azimuths are measured
%   clockwise from north. WindFromDeg follows the meteorological
%   convention and describes where the wind comes from.

arguments
    scenarios table
    options.ModelName (1,1) string = "rocket_6dof_simulation"
    options.StopTime (1,1) double {mustBePositive} = get_base_value("sim_stopTime", 800)
    options.FixedStep (1,1) double {mustBePositive} = 0.02
    options.ShowProgress (1,1) logical = true
end

required = ["LaunchTiltDeg", "LaunchAzimuthDeg", "WindSpeed", "WindFromDeg"];
missing = setdiff(required, string(scenarios.Properties.VariableNames));
if ~isempty(missing)
    error("workshop_run_scenarios:MissingColumns", ...
        "The scenario table requires these columns: %s", strjoin(missing, ", "));
end

nCase = height(scenarios);
if nCase == 0
    simOut = Simulink.SimulationOutput.empty;
    return
end

workshop_configure_cache();

simIn(1, nCase) = Simulink.SimulationInput(char(options.ModelName));
for k = 1:nCase
    initialEuler = workshop_launch_euler( ...
        scenarios.LaunchTiltDeg(k), scenarios.LaunchAzimuthDeg(k));
    windUEN = workshop_wind_vector( ...
        scenarios.WindSpeed(k), scenarios.WindFromDeg(k));

    simIn(k) = Simulink.SimulationInput(char(options.ModelName));
    simIn(k) = simIn(k).setVariable("initial_euler", initialEuler);
    simIn(k) = simIn(k).setVariable("environment_wind", windUEN);
    simIn(k) = simIn(k).setModelParameter( ...
        "StopTime", num2str(options.StopTime), ...
        "FixedStep", num2str(options.FixedStep), ...
        "SimulationMode", "accelerator");
end

simOut = sim(simIn, ...
    "UseFastRestart", "on", ...
    "StopOnError", "on", ...
    "ShowProgress", options.ShowProgress);

% Array simulations can return failed outputs instead of throwing an error.
% Surface the underlying diagnostic before downstream code tries to read logs.
for k = 1:numel(simOut)
    if strlength(string(simOut(k).ErrorMessage)) > 0
        error("workshop_run_scenarios:SimulationFailed", ...
            "Scenario %d failed:\n%s", k, simOut(k).ErrorMessage);
    end
end
end

function value = get_base_value(name, fallback)
if evalin("base", "exist('" + name + "','var')")
    value = evalin("base", name);
else
    value = fallback;
end
end
