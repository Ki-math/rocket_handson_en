function summary = workshop_plots(simOut, titleText)
%WORKSHOP_PLOTS Plot the primary flight-analysis results.
%
%   For a 6DOF result, this function shows trajectory, ground track,
%   altitude, and speed. For the optional 1DOF exercise, it falls back to
%   the roll-control response.

arguments
    simOut
    titleText (1,1) string = "Result"
end

names = string(simOut.logsout.getElementNames());
if any(names == "positionUEN")
    summary = plot_flight_results(simOut, titleText + " - Flight");
else
    plot_roll_control_results(simOut, titleText + " - Roll Control");
    summary = table;
end
end
