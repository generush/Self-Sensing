
%% IPMSM Paramters
% declare all machine parameters global
global Ld Lq Jp bp fs Ts lambda_pm p Rs Ldq

%% Physical Parameters
Ld = 8e-3;                                 % [H] d-axis inductance
Lq = 20e-3;                                % [H] q-axis inductance 
Lqd =0e-3;
Ldq         =   [Ld Lqd ;  Lqd Lq];
Rs = 1.35;                                 % [Ohm] stator resistance
Jp = 3.2e-4; %1.6E-4;                      % [kg-m-s^2] inertia of machine 
bp = 2.2e-4;                                % viscous damping
fs = 10e3;                                  % sample frequency
Ts = 1/fs;                                  % sample time
lambda_pm = 0.13;          					% [V-sec] permanent magnet flux linkage 
p = 2;  