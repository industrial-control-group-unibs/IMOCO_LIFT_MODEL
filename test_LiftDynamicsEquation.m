clear all;close all;clc
addpath('NonLinearElasticModel')
addpath('MotionLaw')
floor_height=3;
num_floors=20;
BuildingHeight=num_floors*floor_height;
min_length=3;
Jp=1;
Jm=0.1;
Mc=600;
Mw=1140;
MaxAcc=1;
MaxVel=2;
MaxJerk=40;

LinearDamping=3000;
LinearStiffness=2100000;

MotorViscousFriction=1;
Rp=0.5;
g=9.806;
mu=0.8;
gearbox=.1;

u=0;
dc=0;
dw=0;

DesiredCabinPosition=0; %meter

[x_eq,u_eq] = LiftEquilibrium(BuildingHeight,LinearStiffness,Mc,Mw,Rp,dc,dw,g,gearbox,min_length,mu,DesiredCabinPosition);


x0=x_eq;
f=@(t,x)LiftDynamicsEquation(u_eq,dc,dw,BuildingHeight,Jp,Jm,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,g,mu,gearbox,min_length,x);


[A,B] = LiftLinearSystem(BuildingHeight,Jm,Jp,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,dc,dw,g,gearbox,min_length,mu,x_eq(9));
poli=eig(A);

C=[0 0 0 0 0 0 0 0 0 1]; % velocity control
%C=[0 0 0 0 0 0 0 0 1 0]; % position control
D=0;
sys=ss(A,B,C,D);
bode(sys)

opt = pidtuneOptions('NumUnstablePoles',sum(real(poli)>0));

wc=5;
ctrl=pidtune(sys,'PIDF',wc,opt);
Ts=1e-3;
