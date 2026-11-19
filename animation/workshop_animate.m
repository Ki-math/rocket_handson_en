function workshop_animate(simOut, options)
%WORKSHOP_ANIMATE  Play the flight animation (helper implementation for students).
%
%   Read vehicle length, CG location, and canopy dimensions from base-workspace design values.
%   Playback speed can be changed with an option.

arguments
    simOut
    options.PlaybackSpeed (1,1) double = 4
    options.FrameRate     (1,1) double = 12
    options.SaveVideo     (1,1) logical = false
end

g = @(n) evalin("base", n);

animate_flight(simOut, ...
    "PlaybackSpeed",  options.PlaybackSpeed, ...
    "FrameRate",      options.FrameRate, ...
    "BodyLength",     g("rocket_length"), ...
    "CgFraction",     g("rocket_cg0(1)") / g("rocket_length"), ...
    "ChuteDiameter",  g("chute_diameter"), ...
    "ChuteRiser",     g("chute_riserLength"), ...
    "SaveVideo",      options.SaveVideo);
end
