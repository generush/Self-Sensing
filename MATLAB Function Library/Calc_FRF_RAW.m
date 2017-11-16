function [MAG,PHA,FREQ] = Calc_FRF_RAW(den, num, T_s)

[input_mag,     input_pha,  freq]   = TwoSidedFFT(den, T_s);
[output_mag,    output_pha, ~   ]   = TwoSidedFFT(num, T_s);
MAG                             = output_mag ./input_mag;
PHA                             =(output_pha - input_pha).*180./pi;
FREQ                            = freq;
end