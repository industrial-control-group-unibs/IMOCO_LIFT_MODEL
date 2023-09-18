clear all;close all;clc;
addpath('..')
BuildingHeight=10;
speed=linspace(-1,1,1e3)';

%% TEST 1 function vs interp1
LUT_position=linspace(0,BuildingHeight,10)';
LUT_Coloumb_friction=randn(length(LUT_position),1);
LUT_viscous_friction=randn(length(LUT_position),1);

for idx=1:1e5
position=rand*BuildingHeight;
velocity=randn;

friction_interp1=LUTfriction_interp1(position,velocity,LUT_position,LUT_Coloumb_friction,LUT_viscous_friction);
friction=LUTfriction(position,velocity,LUT_position,LUT_Coloumb_friction,LUT_viscous_friction);

assert(abs(friction-friction_interp1)<1e-4)
end

%% TEST 2
LUT_position=linspace(0,BuildingHeight,10)';
LUT_Coloumb_friction=10./( (LUT_position-3).^2+1) ;
LUT_viscous_friction=(0.1+0.05./BuildingHeight*LUT_position).*ones(length(LUT_position),1);

simOut=sim('test_friction_lut.slx');

friction=simOut.yout{1}.Values.Data;
position=simOut.yout{2}.Values.Data;
velocity=simOut.yout{3}.Values.Data;
direction=tanh(velocity*100);
% friction=V(position)*vel+C(position*sign(vel)

plot3(position,velocity.*direction,friction.*direction)