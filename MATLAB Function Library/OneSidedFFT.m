%% OneSidedFFT ***(FOR REAL SIGNALS)***
% [Y,Phi,freq] = myfft(y,Ts)
% Description: Performs fft for time signal y(t) with sample time Ts 
% and returns Y(f) and the corresponding frequencies
% Marc Petit
% 06/24/2016

function [Y,Phi,freq] = OneSidedFFT(y,Ts)
    Fs = 1/Ts;
    L  = length(y);
    
    Y = fft(y);
    Phi = unwrap(angle(Y));
    
    P2 = abs(Y/L);
    P1 = P2(1:round(L/2)+1);
    Phi=Phi(1:round(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);
    freq = Fs*(0:round(L/2))/L;
    freq = freq'; % Added so freq matrix has same dimensions as mag and pha
    Y = P1;
    
%     %https://www.mathworks.com/matlabcentral/answers/155349-help-with-fourier-transform-fft
%     N = size(x,1);
%     dt = 1/Fs;
%     t = dt*(0:N-1)';
%     dF = Fs/N;
%     f = dF*(0:N/2-1)';
%     X = fft(x)/N;
%     X = X(1:N/2);
%     X(2:end) = 2*X(2:end);
     
    
end
