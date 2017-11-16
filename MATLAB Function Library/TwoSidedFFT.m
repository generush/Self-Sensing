%% TwoSidedFFT ***(FOR COMPLEX SIGNALS)***
% Description: Performs fft for time signal y(t) with sample time Ts 
% and returns Y(f) and the corresponding frequencies
% Gene Rush
% 02/23/2017

function [Y,Phi,freq] = OneSidedFFT(y,T_s)
    F_s = 1/T_s;
    L = length(y);
    Y = fftshift(fft(y));
    Phi = unwrap(angle(Y));
    Y = abs(Y/L);
    freq = (-L/2:L/2-1)*F_s/L;
    freq = freq'; % Added so freq matrix has same dimensions as mag and pha
    
end

