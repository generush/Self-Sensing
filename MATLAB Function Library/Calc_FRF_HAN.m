
% Hanning Window Example: https://www.mathworks.com/matlabcentral/answers/168656-problem-in-hanning-window-with-fft

% Purpose: Try to understand what tfestimate is doing (used in Calc_FRF_TF_EST)

% Conclusions: Yielded nearly indentical results with function below, which
% manully calculates the number of averages and splices the signals into
% their appropriate length and applies the hanning window.

% Deficiencies: 
%   - The frequency resolution appears to be 2x expected
%   - The phase looks to be incorrect, with values < -1e4 which is huge!,
%     not sure why this is happening

function [MAG,PHA,FREQ] = Calc_FRF_HAN(den, num, lines, T_s)

    total_points        = length(num);  % Number of data points in entire dataset
    f_s                 = 1/T_s;        % Sampling frequency
    f_b                 = f_s/lines;    % Base frequency
    span                = f_s/2;        % Frequency span is fixed by sample frequency
    overlap             = lines/2;      % # of samples overlapped for averaging
    averages            = floor(total_points/(lines-overlap));  % # of averages
    section_points      = total_points/averages;   % Number of data points per section
    
    for i = 1:averages
        section_start                                      = 1 + (i-1)*(section_points);
        section_end                                        = i*(section_points);
        section_den                                        = den(section_start:section_end,:);
        section_num                                        = num(section_start:section_end,:);
        section_han                                        = hanning(section_points);
        
        [input_mag(:,i),     input_pha(:,i),  freq(:,i)]   = TwoSidedFFT(section_den.*section_han, T_s);
        [output_mag(:,i),    output_pha(:,i), ~        ]   = TwoSidedFFT(section_num.*section_han, T_s);
    end

    avg_input_mag = mean(input_mag,2);
    avg_output_mag = mean(output_mag,2);

    avg_input_pha = mean(input_pha,2);
    avg_output_pha = mean(output_pha,2);

    MAG                             = avg_output_mag ./avg_input_mag;
    PHA                             =(avg_output_pha - avg_input_pha).*180./pi;
    FREQ                            = freq(:,1);

end