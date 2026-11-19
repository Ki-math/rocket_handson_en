function euler = workshop_launch_euler(tiltDeg, azimuthDeg)
%WORKSHOP_LAUNCH_EULER Convert launch direction to model Euler angles.
%
%   EULER = WORKSHOP_LAUNCH_EULER(TILTDEG, AZIMUTHDEG) returns the
%   [roll; pitch; yaw] initial condition used by the UEN 6DOF model.
%   TILTDEG is measured from vertical. AZIMUTHDEG is measured clockwise
%   from north: north = 0 deg, east = 90 deg.

arguments
    tiltDeg (1,1) double {mustBeFinite, mustBeGreaterThanOrEqual(tiltDeg, 0), ...
        mustBeLessThan(tiltDeg, 90)}
    azimuthDeg (1,1) double {mustBeFinite}
end

tilt = deg2rad(tiltDeg);
azimuth = deg2rad(azimuthDeg);

up = cos(tilt);
east = sin(tilt) * sin(azimuth);
north = sin(tilt) * cos(azimuth);

pitch = -asin(north);
yaw = atan2(east, up);
euler = [0; pitch; yaw];
end
