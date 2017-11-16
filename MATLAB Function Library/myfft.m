%% MyFFT
% [Y,Phi,freq] = myfft(y,Ts)
% Description: Performs fft for time signal y(t) with sample time Ts 
% and returns Y(f) and the corresponding frequencies
% Marc Petit
% 06/24/2016

function [Y,Phi,freq] = myfft(y,Ts)
    Fs = 1/Ts;
    L  = length(y);
    
    Y = fft(y);
    Phi = unwrap(angle(Y));
    
    P2 = abs(Y/L);
    P1 = P2(1:round(L/2)+1);
    Phi=Phi(1:round(L/2)+1);
    P1(1:end) = 2*P1(1:end);    %CHANGED, WAS: P1(2:end-1) = 2*P1(2:end-1);
    freq = Fs*(1:round(L/2)+1)/L;
    Y = P1;
end
