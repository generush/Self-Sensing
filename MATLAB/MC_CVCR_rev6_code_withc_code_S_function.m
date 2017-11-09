%% Closed-Loop Permanent Magnet Synchronous Machine (PMSM) Simulation

%%%%%%%%%%%%%%%%%%%%%%%%%% DOCUMENTION OF ISSUES %%%%%%%%%%%%%%%%%%%%%%%%%%

% Notes about This Code
% - There is a strange peak in CT when higher fundamental frequencies are
% inputted, but this is not a problem for actual operation, which is in the
% low speed regime.

% - Wrote code to enable chirp that has negative and positive frequencies,
% but I stopped using that and started using the Simulink blocks for chirp
% and random noise (the chirp doesn't have negative frequencies.

% - Dynamic Stiffness plots don't really help tell the story...but I left
% the code in there.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Questions for Tim

%   - why do we tune differently than specified in the Discrete CVCR paper?
%   - 

%% Test Simulation

clc
clear all
close all

T_sim = 1e-6;                                   %[s]

param_577_Lab   = 0;
param_Research  = 1;
param_debug     = 2;

run = param_Research;

switch run
    case param_577_Lab
        % Define Sampling
        f_s         = 20000;                    %[Hz]
        T_s         = 1/f_s;                    %[s]

        % Machine Parameters
        P               = 8;                    %[# poles]
        Lambda_pm       = 0.0065;               %[Wb]
        K_T             = 3/2*(P/2)*Lambda_pm;  %[Nm/A]
        R_s             = 0.459;                %[Ohms]
        L_s             = 0.24e-3;              %[H]
        J_p             = 2.048e-5;             %[kgm^2]
        b_p             = 1.3e-4;               %[Nm/s]
        T_u             = 0.015;                %[Nm]

        % Test Machine - Current Regulator
        BW_CVCR         = 500;                  %[Hz]
        K_p             = 2*pi*BW_CVCR*L_s;     %[Ohms]
        K_i             = 2*pi*BW_CVCR*R_s;     %[Ohms-rad/s]

        % Test Machine - Motion Control
        BWa1            = 50;                   %[Hz]
        BWb1            = BWa1/5;               %[Hz]
        BWc1            = BWb1/5;               %[Hz]

        za1             = exp(-2*pi*BWa1*T_s);
        zb1             = exp(-2*pi*BWb1*T_s);
        zc1             = exp(-2*pi*BWc1*T_s);

        b_a             = (-za1*zb1*zc1+1)*J_p/(T_s);                               %[Nm/rad] = [Nm]
        K_sa            = ((-(za1*zb1+za1*zc1+zb1*zc1)+3)*J_p-2*T_s*b_a)/((T_s)^2);	%[Nms/(rad/s)] = [Nms]
        K_isa           = ((-(za1+zb1+zc1)+3)*J_p-T_s*b_a-(T_s)^2*K_sa)/((T_s)^3);  %[Nm/(rad/s^2)] = [Nms^2]

        % Motion Command State Filter
        Ksf1            = 1000;   % about 200 Hz BW
        Ksf2            = 4000;
        NegSpeedLimit   = -600;   %[rad/s]
        PosSpeedLimit   =  600;   %[rad/s]

    case param_Research
        % Define Sampling
        f_s         = 20000;                    %[Hz]
        T_s         = 1/f_s;                    %[s]

        % Machine Parameters
        P               = 8;                    %[# poles]
        K_e             = 0.008718957000640;    %[?]
        K_T             = 3/2*K_e;              %[Nm/A]
        Lambda_pm       = K_e/(P/2);            %[Wb]           (0.002) Tim's Thesis pg 131
        R_s             = 0.116959358962624;    %[Ohms]         (0.1169) (Stator Resistance) Tim's Thesis pg 139
        L_s             = 44.5e-6;              %[H]            (46.71e-6+42.28e-6)/2 (Phase Inductance) Tim's Thesis pg 139
        L_qs            = 46.71e-6;             %[H]
        L_ds            = 42.28e-6;             %[H]
        J_p             = 6.09e-5;              %[kgm^2]
        b_p             = 1e-4;                 %[Nm/s]
        T_u             = 0.047;                %[Nm]

        % Test Machine - Current Regulator
        BW_CVCR         = 250;                  %[Hz]           (250) Tim's Thesis pg 141
        K_p             = 2*pi*BW_CVCR*L_s;     %[Ohms]         (based on discrete CVCR paper) K_p = abs(R*exp(-R/L*T_s)*(z_desired - 1)/(1 - exp(-R/L*T_s))) %Tune K_i so controller zero cancels plant pole
        K_i             = 2*pi*BW_CVCR*R_s;     %[Ohms-rad/s]   (based on discrete CVCR paper) K_i = K_p*(1 - exp(-R/L*T_s))/(T_s*exp(-R/L*T_s)) %Tune K_p to satisfy magnitude condition


        % Test Machine - Motion Control
        BWa1            = 5;                   %[Hz]
        BWb1            = BWa1/5;               %[Hz]
        BWc1            = BWb1/5;               %[Hz]

        za1             = exp(-2*pi*BWa1*T_s);
        zb1             = exp(-2*pi*BWb1*T_s);
        zc1             = exp(-2*pi*BWc1*T_s);

        b_a             = (-za1*zb1*zc1+1)*J_p/(T_s);                               %[Nm/rad] = [Nm] z root corresponding to desired BW
        K_sa            = ((-(za1*zb1+za1*zc1+zb1*zc1)+3)*J_p-2*T_s*b_a)/((T_s)^2); %[Nms/(rad/s)] = [Nms] Tune K_i so controller zero cancels plant pole
        K_isa           = ((-(za1+zb1+zc1)+3)*J_p-T_s*b_a-(T_s)^2*K_sa)/((T_s)^3);  %[Nm/(rad/s^2)] = [Nms^2] Tune K_p to satisfy magnitude condition

        % Motion Command State Filter
        Ksf1            = 1000;   % about 200 Hz BW
        Ksf2            = 4000;
        NegSpeedLimit   = -600;   %[rad/s]
        PosSpeedLimit   =  600;   %[rad/s]

end

%% Run Simulink Model
endtime         = 5; %30 used for T/w DS      %[s]
mdl             = 'MC_CVCR_rev6_sim_withc_code_S_function';
set_param(mdl,'StopTime',num2str(endtime));
sim(mdl);

%% Process Simulation Data

time = logsout{1}.Values.Time; % Acquire sample time data
i = 1;

while 1
    try
    name_temp = genvarname(strcat('log_',logsout{i}.Name));	% generate variable name
    data_temp = squeeze(logsout{i}.Values.Data);        	% acquire variable data
    eval([name_temp '= data_temp;']);                       % assign variable name
    i = i + 1;
    
    clear name_temp; clear data_temp;                       % clear temp variables
    catch
        clear name_temp; clear data_temp;
        break
    end
end

%% Time Domain Plottings

figure('Name','t, omega'); hold on; grid on; plottools
plot(time,log_omega_rm_star);
plot(time,log_omega_rm);

figure('Name','t, theta'); hold on; grid on; plottools
plot(time,log_theta_rm_star);
plot(time,log_theta_rm);

%% Calculate the Number of Lines for FRFs

averages = 2;
lines = floor(length(log_T_s)/((averages+1)*0.50)); %assuming 50% overlap (Derived 10/7/2017)

%% Calculate (Speed) CT FRF (WINDOW/AVG)
% This calculation outputs correct FRFs for both mag and pha, verified with single sine tests

[CT_Est_MAG, CT_Est_PHA, CT_Est_COH, CT_Est_FREQ] = Calc_FRF_TF_EST(log_omega_rm_avg_star, log_omega_rm_avg, lines, T_s, 'onesided');

figure('Name','Speed CT MAG'); hold on; grid on; plottools
    title('FRF Magnitude');
    xlabel('Frequency [Hz]'); ylabel('Magnitude');
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'linear')
    xlim([1e-2,1e3]); ylim([0,2]);
    plot(CT_Est_FREQ, CT_Est_MAG,'r*','MarkerSize',8);
   
figure('Name','Speed CT PHA'); hold on; grid on; plottools
    title('FRF Phase');
    xlabel('Frequency [Hz]'); ylabel('Phase');
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'linear')
    xlim([1e-2,1e3]); ylim([-90,90])
    plot(CT_Est_FREQ, CT_Est_PHA,'r*','MarkerSize',8);

figure('Name','Speed CT COH'); hold on; grid on; plottools
    title('FRF Coherence');
    xlabel('Frequency [Hz]'); ylabel('Coherence'); 
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'linear')
    xlim([1e-2,1e3]); ylim([0,1]);
    plot(CT_Est_FREQ, CT_Est_COH,'r*','MarkerSize',8);
   
%% Calculate (Speed) DS FRF (WINDOW/AVG)
% This calculation outputs correct FRFs for both mag and pha, verified with single sine tests

[DS_Est_MAG, DS_Est_PHA, DS_Est_COH, DS_Est_FREQ] = Calc_FRF_TF_EST(log_omega_rm_avg, log_T_d, lines, T_s, 'onesided');

figure('Name','Speed DS MAG'); hold on; grid on; plottools
    title('FRF Magnitude');
    xlabel('Frequency [Hz]'); ylabel('Magnitude');
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log')
    xlim([1e-2,1e3]); ylim([0,2]);
    plot(DS_Est_FREQ, DS_Est_MAG,'r*','MarkerSize',8);
   
figure('Name','Speed DS PHA'); hold on; grid on; plottools
    title('FRF Phase');
    xlabel('Frequency [Hz]'); ylabel('Phase');
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'linear')
    xlim([1e-2,1e3]); ylim([-90,90])
    plot(DS_Est_FREQ, DS_Est_PHA,'r*','MarkerSize',8);

figure('Name','Speed DS COH'); hold on; grid on; plottools
    title('FRF Coherence');
    xlabel('Frequency [Hz]'); ylabel('Coherence'); 
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'linear')
    xlim([1e-2,1e3]); ylim([0,1]);
    plot(DS_Est_FREQ, DS_Est_COH,'r*','MarkerSize',8);
    
% Plot DS MAG with (jw)^2 weighting
figure('Name','Speed DS MAG jw'); hold on; grid on; plottools
    title('FRF Magnitude');
    xlabel('Frequency [Hz]'); ylabel('Magnitude');
    set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log')
    xlim([1e-2,1e3]); ylim([1e-5,1e5]);
    plot(DS_Est_FREQ, DS_Est_MAG.*(2*pi*DS_Est_FREQ).^2 ,'r*','MarkerSize',8);
    plot(DS_Est_FREQ, DS_Est_MAG.*(2*pi*DS_Est_FREQ)    ,'b*','MarkerSize',8);
    plot(DS_Est_FREQ, DS_Est_MAG                        ,'k*','MarkerSize',8);
    plot(DS_Est_FREQ, DS_Est_MAG./(2*pi*DS_Est_FREQ)    ,'g*','MarkerSize',8);
    
%% Calculate (Current) CT FRF (WINDOW/AVG)
% This calculation outputs correct FRFs for both mag and pha, verified with single sine tests

[CT_Est_MAG, CT_Est_PHA, CT_Est_COH, CT_Est_FREQ] = Calc_FRF_TF_EST(log_i_q_e_star - j*log_i_d_e_star, log_i_q_e - j*log_i_d_e, lines, T_s, 'twosided');

figure('Name','Speed CT MAG'); hold on; grid on; plottools
    title('FRF Magnitude');
    xlabel('Frequency [Hz]'); ylabel('Magnitude');
    set(gca, 'XScale', 'linear'); set(gca, 'YScale', 'linear')
    xlim([-1e3,1e3]); ylim([0,2]);
    plot(CT_Est_FREQ, CT_Est_MAG,'r.','MarkerSize',8);
   
figure('Name','Speed CT PHA'); hold on; grid on; plottools
    title('FRF Phase');
    xlabel('Frequency [Hz]'); ylabel('Phase');
    set(gca, 'XScale', 'linear'); set(gca, 'YScale', 'linear')
    xlim([-1e3,1e3]); ylim([-90,90])
    plot(CT_Est_FREQ, CT_Est_PHA,'r.','MarkerSize',8);

figure('Name','Speed CT COH'); hold on; grid on; plottools
    title('FRF Coherence');
    xlabel('Frequency [Hz]'); ylabel('Coherence'); 
    set(gca, 'XScale', 'linear'); set(gca, 'YScale', 'linear')
    xlim([-1e3,1e3]); ylim([0,1]);
    plot(CT_Est_FREQ, CT_Est_COH,'r.','MarkerSize',8);
%% Misc Code
    
% z       = tf('z',T_s);
% Gc      = K_p + K_i*T_s*z/(z - 1);
% Gp      = 1/R*(1 - exp(-R/L*T_s))/(z - exp(-R/L*T_s));
% OLTF    = tf(Gc*Gp,T_s)
% 
% figure(q); q=q+1; hold on; grid on; plottools
% rlocus(OLTF) %pzmap(Gc*Gp/(1+Gc*Gp))
