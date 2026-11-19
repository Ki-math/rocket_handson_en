function workshop_setup()
%WORKSHOP_SETUP  Prepare the workshop environment.
%
%   Add paths, configure signal logging, and check required parameters.
%   Call once after defining parameters in the Live Script.
%
%   Parameters are read from the base workspace used by the Live Script.

check_required_parameters();

sync_model_signal_logging("rocket_6dof_simulation", false);

print_flight_condition();
end


function check_required_parameters()
%CHECK_REQUIRED_PARAMETERS  Check that all model variables are defined.
%   List extracted from the 6DOF model using Simulink.findVars. Update this
%   list when model variable names change to catch mismatches before execution.
required = [ ...
    "initial_position", "initial_velocityBody", "initial_euler", "initial_pqr", ...
    "rocket_mass0", "rocket_massDry", "rocket_inertia0", "rocket_inertiaDry", ...
    "CG", "CP", "aero_refArea", "aero_refLength", "Fth", "Isp", ...
    "environment_g", "environment_wind", ...
    "Cx_0", "Cy_0", "Cz_0", "Cy_b", "Cz_a", "Cm_a", "Cm_q", "Cn_b", "Cn_r", ...
    "Cl_p", "Cl_delta", ...
    "chute_CdS", "chute_inflateTau", "chute_attachBody", "chute_riserLength", ...
    "fin_maxDeflection", "fin_maxRate", "fin_actuatorTau", ...
    "sensor_gyroTau", "sensor_attitudeTau", ...
    "controller_outer_Kp", "controller_outer_maxRateCmd", ...
    "controller_inner_Kp", ...
    "command_rollStep", "command_stepTime", "propulsion_cutoffTime", ...
    "sim_stopTime", "sim_fixedStep"];

missing = string.empty;
for name = required
    if evalin("base", sprintf("exist('%s','var')", name)) ~= 1
        missing(end + 1) = name; %#ok<AGROW>
    end
end
if ~isempty(missing)
    error("workshop_setup:MissingParameters", ...
        "Undefined parameters:\n  %s", strjoin(missing, newline + "  "));
end
end


function print_flight_condition()
%PRINT_FLIGHT_CONDITION  Print flight conditions derived from the design values.
g = @(n) evalin("base", n);
staticMargin = g("CG(1)") - g("CP(1)");
fprintf("--- Design conditions ---\n");
if evalin("base", "exist('launchTiltDeg','var')")
    fprintf("  Launch direction : %.1f deg from vertical / azimuth %.1f deg\n", ...
        g("launchTiltDeg"), g("launchAzimuthDeg"));
end
if evalin("base", "exist('windSpeed','var')")
    fprintf("  Wind             : %.1f m/s / from %.1f deg\n", ...
        g("windSpeed"), g("windFromDeg"));
end
fprintf("  Reference area : %.6f m^2 (diameter %.3f m)\n", g("aero_refArea"), g("rocket_diameter"));
fprintf("  Static margin  : %.3f m (%.2f calibers)\n", staticMargin, staticMargin / g("rocket_diameter"));
fprintf("  Thrust / burn  : %.0f N / %.1f s (mdot %.3f kg/s)\n", ...
    g("Fth"), g("propulsion_burnTime"), g("Fth") / g("Isp"));
fprintf("  Canopy CdS     : %.2f m^2 -> terminal speed %.1f m/s\n\n", ...
    g("chute_CdS"), sqrt(2 * g("rocket_massDry") * g("environment_g") / (1.225 * g("chute_CdS"))));
end
