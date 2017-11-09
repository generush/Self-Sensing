% Description:
% Simple IPMSM Model to Test AIX Code
% Author: Marc Petit
% Created: 01/27/2016
% Last Modified: 04/06/2016

clc
close all;
clear all;
%% Initialisation

global mdl
global Ld Lq  Jp bp fs Ts lambda_pm p Rs Ldq


E = [1, 0; 0, 1];
j = [0, -1; 1, 0];

mdl = 'MC_CVCR_rev6_sim_withc_code_S_function';

open_system(mdl);

IPMSM_param;           % Physical Parameters
Td_amp = 0;

save ws.mat;
hws = get_param(bdroot, 'modelworkspace');
hws.reload;

