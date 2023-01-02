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

u_test=0.99*u_eq;
f=@(x)LiftDynamicsEquation(u_test,dc,dw,BuildingHeight,Jp,Jm,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,g,mu,gearbox,min_length,x);
ode=@(t,x)f(x); % ode45 needs time as input



x0=x_eq;
x0(1)=x0(1)+.1; % add small perturbation
Tsim=100;


options = odeset('RelTol',1e-5);%,'MaxStep',1e-3);
[t_rk45,x_rk45] = ode45(ode,[0 Tsim],x0,options);

x=x0;


subplot(211)
plot(t_rk45,x_rk45(:,7),'b')
xlabel('time')
ylabel('position cabin')

subplot(212)
plot(t_rk45,x_rk45(:,[1]),'b')
xlabel('time')
ylabel('position counterweight')


Ts=1e-3;
t=(0:Ts:Tsim)';
%% Euler

if 0
    x_euler=zeros(length(t),10);
    for idx=1:length(t)
        if idx==1
            x=x0;
        else
            dx=f(x);
            x=x+dx*Ts;
        end
        x_euler(idx,:)=x'; % store in row
    end


    subplot(211)
    hold on
    plot(t,x_euler(:,[7]),'--r')

    subplot(212)
    hold on
    plot(t,x_euler(:,[1]),'--r')
end

%% RK4
x_rk4=zeros(length(t),10);
for idx=1:length(t)
    if idx==1
        x=x0;
    else
        dx1=f(x);
        dx2=f(x+dx1*0.5*Ts);
        dx3=f(x+dx2*0.5*Ts);
        dx4=f(x+dx3*Ts);
        dx=(dx1+2*dx2+2*dx3+dx4)/6.0;
        x=x+dx*Ts;
    end
    x_rk4(idx,:)=x'; % store in row
end


subplot(211)
hold on
plot(t,x_rk4(:,[7]),'--k')

subplot(212)
hold on
plot(t,x_rk4(:,[1]),'--k')

return

%% RK2

for idx=1:length(t)
    if idx==1
        x=x0;
    else
        dx1=f(x);
        dx2=f(x+dx1*0.5*Ts);
        %dx3=f(x+dx2*0.5*Ts);
        %dx4=f(x+dx3*Ts);
        dx=(dx1+2*dx2)/3.0;
        x=x+dx*Ts;
    end
    x_rk2(idx,:)=x'; % store in row
end


subplot(211)
hold on
plot(t,x_rk2(:,[7]),'--m')

subplot(212)
hold on
plot(t,x_rk2(:,[1]),'--m')