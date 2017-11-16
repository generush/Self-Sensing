% INPUTS:
%  lines: Number of lines for FFT, YOU CAN VARY THIS TO CHANGE f_b & # of averages

% Derived 10/7/2017: length(num) = lines +  (averages - 1)*(lines - overlap)
function [MAG, PHA, COH, FREQ] = Calc_FRF_TF_EST(den, num, lines, T_s, freqrange)

    f_s                 = 1/T_s;        % Sampling frequency
    f_b                 = f_s/lines;    % Base frequency
    span                = f_s/2;        % Frequency span is fixed by sample frequency
    overlap             = lines/2;      % # of samples overlapped for averaging
    averages            = floor((length(num)-overlap)/(lines-overlap));  % # of averages (derived 10/7/2017)   

    
    [RESP, FREQ]        = tfestimate(den,num,hanning(lines),overlap,lines,1/T_s);
    [COH, FREQ]         = mscohere(den,num,hanning(lines),overlap,lines,1/T_s);

    RESP                = fftshift(RESP);
    COH                 = fftshift(COH);

    MAG                 = abs(RESP);
    PHA                 = angle(RESP)*180/pi;
    FREQ                = FREQ - max(FREQ)/2;

end