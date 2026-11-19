function sync_model_signal_logging(modelName, saveModel)
%SYNC_MODEL_SIGNAL_LOGGING  Standardize suffix-free signal names and enable logging.
%
%   SYNC_MODEL_SIGNAL_LOGGING(MODELNAME) operates on existing named lines
%   without rewiring the model:
%     1. Rename legacy signals with unit suffixes (_deg / _dps / _m, etc.).
%     2. Enable signal logging and apply logging_decimation.
%
%   SYNC_MODEL_SIGNAL_LOGGING(MODELNAME, TRUE) also saves the model.
%   The default does not save. Saving a newer model in an older Simulink
%   release downgrades its format, so saving is an explicit choice.
%
%   All model units are SI plus rad. Unit-free signal names avoid a hidden
%   mismatch between the name and the actual units.

arguments
    modelName (1,1) string
    saveModel (1,1) logical = false
end

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

set_param(modelName, "SignalLogging", "on", "SignalLoggingName", "logsout");

decimation = 1;
if evalin("base", "exist('logging_decimation','var')")
    decimation = evalin("base", "logging_decimation");
end

map = signal_rename_map(modelName);
for k = 1:size(map, 1)
    apply_signal(modelName, map(k, 1), map(k, 2), decimation);
end

if saveModel
    save_system(modelName);
end
end


function map = signal_rename_map(modelName)
%SIGNAL_RENAME_MAP  [oldName, newName] mapping; leave already-renamed signals unchanged.
switch string(modelName)
    case "rocket_6dof_simulation"
        map = [ ...
            "positionUEN_m",    "positionUEN"; ...
            "velocityBody_mps", "velocityBody"; ...
            "quaternionBI",     "quaternionBI"; ...
            "euler",            "euler"; ...
            "pqr",              "pqr"; ...
            "qbar_Pa",          "qbar"; ...
            "Mach",             "Mach"; ...
            "forcesBody_N",     "forcesBody"; ...
            "momentsBody_Nm",   "momentsBody"; ...
            "phiCmd_deg",       "phiCmd"; ...
            "pCmd_dps",         "pCmd"; ...
            "deltaCmd_deg",     "deltaCmd"; ...
            "delta_deg",        "delta"];

    case "roll_1dof_model"
        map = [ ...
            "phiCmd_deg",   "phiCmd"; ...
            "pCmd_dps",     "pCmd"; ...
            "deltaCmd_deg", "deltaCmd"; ...
            "delta_deg",    "delta"; ...
            "phi_deg",      "phi"; ...
            "p_dps",        "p"];

    otherwise
        error("sync_model_signal_logging:UnsupportedModel", ...
            "Unsupported model: %s", modelName);
end
end


function apply_signal(modelName, oldName, newName, decimation)
%APPLY_SIGNAL  Use the new signal name, or rename the legacy line if needed.
lineHandle = find_line_by_name(modelName, newName);
if isempty(lineHandle)
    lineHandle = find_line_by_name(modelName, oldName);
end
if isempty(lineHandle)
    warning("sync_model_signal_logging:SignalNotFound", ...
        "No line named %s (legacy name %s) was found in %s.", newName, oldName, modelName);
    return
end

set_param(lineHandle, "Name", char(newName));
Simulink.sdi.markSignalForStreaming(lineHandle, "on");
try
    set_param(lineHandle, "DataLoggingDecimation", num2str(decimation));
catch
end
end


function lineHandle = find_line_by_name(modelName, signalName)
%FIND_LINE_BY_NAME  Find a named line; return the parent line for a branch.
lineHandle = [];
allLines = find_system(modelName, "FindAll", "on", "type", "line");
for k = 1:numel(allLines)
    try
        if strcmp(get_param(allLines(k), "Name"), char(signalName))
            if get_param(allLines(k), "LineParent") == -1
                lineHandle = allLines(k);
                return
            end
            if isempty(lineHandle)
                lineHandle = allLines(k);
            end
        end
    catch
    end
end
end
