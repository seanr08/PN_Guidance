function [K_delta, M_delta, V, I_yy, omega_a, zeta_a, TF_par] = getAutopilot()

%% 1) Define known parameters
zeta_a      = 0.7;        % servo damping ratio
omega_a     = 50;        % servo natural freq (rad/s)
TR          = 0.2;        % desired rise time (s)

a           = 1125;               % ft/s
rho         = 14.96e-4;           % slug/ft^3
S           = 0.059*2;              % ft^2
l_ref       = 0.277;             % ft
Cm_delta    = 0.05;               % per rad
V           = 2.2*a;              % ft/s        
q           = 0.5 * rho * V^2;    % lb/ft^2

M_delta     = Cm_delta * q * S * l_ref;   % [ft·lb/rad]
I_yy        = 0.0621;                     % [slug·ft²]     pitch moment of inertia

%% 2) Compute K_delta (accel→deflection gain)
K_delta = I_yy / (M_delta * V);

%% 3) Build the full‐order plant
s       = tf('s');
G_servo = omega_a^2 / ( s^2 + 2*zeta_a*omega_a*s + omega_a^2 );
G_aero  = (M_delta * V) / ( I_yy * s );
G_full  = K_delta * G_servo * G_aero;
%{
%% 4) Compute the “fastest” pure‑P gain
Kp = (sqrt(2)*I_yy*zeta_a*omega_a) / (K_delta*M_delta*V);

%% 5) Define PID terms
Ki = 0.1 * Kp;    % integral gain (start small)
Kd = 0.01 * Kp;   % derivative gain (start tiny)
%}
%% 2) Desired bandwidth for Tr=0.1 s
wc = 1.8/TR;   % rad/s

%% 3) Tune a PID to hit that bandwidth
%   pidtune returns both the controller and info struct
[C_pid, info] = pidtune(G_full, 'PID', wc);

T_cl = feedback(C_pid*G_full, 1)

fprintf('Stability Check: %.4f\n', info.Stable);
fprintf('Crossover Frequency: %.4f\n', info.CrossoverFrequency);
fprintf('Phase Margin: %.4f\n', info.PhaseMargin);
fprintf('Kp: %.4f\n', C_pid.Kp);
fprintf('Ki: %.4f\n', C_pid.Ki);
fprintf('Kd: %.4f\n', C_pid.Kd);

figure;
step(G_full);
grid on;
title('Open Loop Response')

figure;
step(T_cl);
grid on;
title('Closed Loop Response')

figure;
rlocus(G_full);
grid on;
title('Open Loop Root Locus')

figure;
rlocus(T_cl);
grid on;
title('Closed Loop Root Locus')

T_cl = blkdiag(T_cl,T_cl,T_cl);

eigs = eig(T_cl);
fprintf('Closed Loop Eigenvalues: %.4f\n', eigs);

[ Acl, Bcl, Ccl, Dcl ] = ssdata( ss(T_cl) )
TF_par = struct( 'A',Acl, 'B',Bcl, 'C',Ccl, 'D',Dcl );

end