function simOut = workshop_simulate(modelName, stopTime)
%WORKSHOP_SIMULATE  Run the model once (helper implementation for students).
%
%   SIMOUT = WORKSHOP_SIMULATE(MODELNAME) runs until sim_stopTime.
%   SIMOUT = WORKSHOP_SIMULATE(MODELNAME, STOPTIME) overrides the stop time.

if nargin < 2
    stopTime = evalin("base", "sim_stopTime");
end

in = Simulink.SimulationInput(char(modelName));
in = in.setModelParameter("StopTime", num2str(stopTime));
simOut = sim(in);
end
