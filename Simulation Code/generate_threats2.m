function [RT, VT] = generate_threats2(numThreats, altRange, velRange)
%GENERATE_THREATS Randomly generate threat missile initial conditions.
%   [RT, VT] = generate_threats(numThreats, altRange, velRange) returns:
%     RT: numThreats x 3 matrix of initial positions [x y z] (ft)
%     VT: numThreats x 3 matrix of initial velocities [vx vy vz] (ft/s)
%
%   Inputs:
%     numThreats - number of threat missiles to generate
%     altRange   - [minAlt, maxAlt] in feet (positive)
%     velRange   - [minV, maxV] missile speed range in ft/s
%
%   Each missile is placed at a random ground-range between 10 and 13 miles
%   from the origin (launch site), at a random bearing. Altitude is uniform
%   in altRange. Velocity magnitude is uniform in velRange. Each missile
%   is aimed at a random point within 3 miles of the origin, so vz is
%   downward (negative).

% Constants
mile_ft = 5280;
R_gr_min = 12 * mile_ft;
R_gr_max = 13 * mile_ft;
R_aim_max = 5 * mile_ft;

% Preallocate outputs
RT = zeros(numThreats,3);
VT = zeros(numThreats,3);

% Generate positions
theta = 2*pi * rand(numThreats,1);             % random bearing
r_gr   = R_gr_min + (R_gr_max - R_gr_min) * rand(numThreats,1);
RT(:,1) = r_gr .* cos(theta);                  % x positions
RT(:,2) = r_gr .* sin(theta);                  % y positions
RT(:,3) = altRange(1) + diff(altRange) * rand(numThreats,1);  % z altitudes

% Generate aim points within 3-mile radius at ground (z=0)
phi = 2*pi * rand(numThreats,1);
s = R_aim_max * sqrt(rand(numThreats,1));      % uniform in circle
aim_x = s .* cos(phi);
aim_y = s .* sin(phi);
aim_z = zeros(numThreats,1);

% Compute direction vectors from RT to aim point
D = [aim_x - RT(:,1), aim_y - RT(:,2), aim_z - RT(:,3)];
% Normalize
D_norm = sqrt(sum(D.^2,2));
U = D ./ D_norm;

% Generate speeds and velocities
speeds = velRange(1) + diff(velRange) * rand(numThreats,1);
VT = U .* speeds;

RT=RT';
VT=VT';

% Ensure negative z-velocity
% (should be guaranteed since aim_z=0 and RT_z>0 => U(:,3)<0)
end