clear all;close all;clc

BuildingHeight=40;
min_length=3;
Jp=1;
Jm=0.1;
Mc=600;
Mw=1140;

LinearDamping=10;
LinearStiffness=2100000;

MotorViscousFriction=1;
Rp=0.5;
g=9.806;
mu=0.8;
gearbox=.1;


dc=0;
dw=0;




motor_range=linspace(0,BuildingHeight/Rp/gearbox,20)';


%C=[0 0 0 0 0 0 0 0 0 1]; % velocity control
C=[0 0 0 0 0 0 0 0 1 0]; % position control
D=0;

for idx=1:length(motor_range)
    [x_eq,u_eq] = LiftEquilibrium(BuildingHeight,LinearStiffness,Mc,Mw,Rp,dc,dw,g,gearbox,min_length,mu,motor_range(idx));
    x(idx,:)=x_eq';
    u(idx,1)=u_eq;

    [A,B] = LiftLinearSystem(BuildingHeight,Jm,Jp,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,dc,dw,g,gearbox,min_length,mu,x_eq(9));


% 
%     sys=ss(A,B,C,D);
%     bode(sys)
%     hold on
end




