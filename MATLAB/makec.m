%makectrl.m
clc;
clear mex; 
cd('../AIX_CODE/');
mex -g ctrl_sil.cpp Encoder.cpp AnalogInput.cpp ComplexVar.cpp FOC.cpp MotionController.cpp MotionObserver.cpp ;
copyfile('../AIX_CODE/ctrl_sil.mexw32','../MATLAB')