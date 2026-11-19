function windUEN = workshop_wind_vector(speed, fromAzimuthDeg)
%WORKSHOP_WIND_VECTOR Convert meteorological wind to a UEN velocity.
%
%   WINDUEN = WORKSHOP_WIND_VECTOR(SPEED, FROMAZIMUTHDEG) returns
%   [Up; East; North] in m/s. FROMAZIMUTHDEG is the direction the wind
%   comes from, clockwise from north. For example, 270 deg is a west wind.

arguments
    speed (1,1) double {mustBeFinite, mustBeNonnegative}
    fromAzimuthDeg (1,1) double {mustBeFinite}
end

windUEN = [0; ...
    -speed * sind(fromAzimuthDeg); ...
    -speed * cosd(fromAzimuthDeg)];
end
