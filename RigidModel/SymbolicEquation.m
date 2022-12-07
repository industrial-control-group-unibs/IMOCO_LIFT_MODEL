%% PARAMETERS, STATE AND INPUTS
% System input
syms u  % motor torque
syms dc % disturbance on the cabin side
syms dw % disturbance on the counterweight side

% parameters
syms Mc % mass of the cabin
syms Mw % mass of the counterweight
syms Jp % inertia of the pulley (and all the elements rigidly connected with it)
syms Jm % inertia of the motor (and all the elements rigidly connected with it)
syms gearbox % gearbox ratio (outer speed/inner speed)
syms Rp  % radius of the pulley
syms mu  % linear density (mass of a 1 meter of rope)
syms BuildingHeight % movement range
syms MotorViscousFriction % motor friction
syms g % gravitational acceleration
syms min_length % minimum distance between the pulley and the cabin 


% System state (overside to keep the same dimension of the elastic one)
% x1 : position of the counterweight
% x2 : velocity of the counterweight
% % % x3 : position of the rope mass (lumped in the middle of the robot) on the
% % % counterweight side
% % % x4 : velocity of the rope mass (lumped in the middle of the robot) on the
% % % counterweight side
% % % x5 : position of the rope mass (lumped in the middle of the robot) on the
% % % cabin side
% % % x6 : velocity of the rope mass (lumped in the middle of the robot) on the
% % % cabin side
% x7 : position of the cabin
% x8 : velocity of the cabin
% x9 : angular position of the motor
% x10: angular velocity of the motor
x=sym('state',[10,1]);


% Initia conditions: assumption -> when x9=0 the cabin is in 0


total_inertia_motor_side=Jp*gearbox^2+Jm;
h_pulley=BuildingHeight+min_length;
%% Sistema definito come dx/dt=f(x,u) con x = [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10]

pulley_angle=x(9)*gearbox;
pulley_speed=x(10)*gearbox;
pulley_linear_speed=pulley_speed*Rp;



nominal_x7=pulley_angle*Rp; % nominal cabin position
nominal_x5=(h_pulley+nominal_x7)/2; % rope is in the middle between the pulley and the cabin
nominal_x8=pulley_speed;
nominal_x6=nominal_x8/2;


nominal_x1=BuildingHeight-nominal_x7; % counterweight is up when cabin is down
nominal_x3=(h_pulley+nominal_x1)/2; % rope is in the middle between the pulley and the counterweight
nominal_x2=-pulley_speed;
nominal_x4=nominal_x2/2;


lc=h_pulley-nominal_x7; % rope length on cabin side (ignoring spring deflection)
lw=h_pulley-nominal_x1; % rope length on counterweight side  (ignoring spring deflection)

m_rw=mu*lw; % rope mass on cabin side
m_rc=mu*lc; % rope mass on cabin side

syms angular_acceleration % angular acceleration

pulley_acc=angular_acceleration*gearbox;
acc_c=pulley_acc*Rp;
acc_w=-pulley_acc*Rp;


F_pulley=dc-(Mc+m_rc)*g-(Mc+m_rc)*acc_c;
F_counterweight=dw-(Mw+m_rw)*g-(Mw+m_rw)*acc_w;

torque_of_the_ropes_on_the_pulley=( -F_pulley + F_counterweight )*Rp;

eq = total_inertia_motor_side*angular_acceleration==u+torque_of_the_ropes_on_the_pulley -  x(10)*MotorViscousFriction;

Dx10=solve(eq,angular_acceleration);
f(1)=x(10);
f(2)=Dx10;

f=simplify(f);

%% Compute equilibrium for a given x7
% velocity x2,x4,x6,x8,x10 are null at the equilibrium
% position x1,x3,x5,x7 are unkwown, while x9 is given
% ueq  is unkwown
% dc and dw are supposed null

eq_equilibrium = f == 0;

solution_eq = solve(eq_equilibrium,[u,x(10)]); 

x_eq = [x(9);solution_eq.state10];%[solution_eq.state1; solution_eq.state2; solution_eq.state3; solution_eq.state4; solution_eq.state5; solution_eq.state6; solution_eq.state7; solution_eq.state8; x(9); solution_eq.state10];
u_eq = solution_eq.u;

x_eq=simplify(x_eq);
u_eq=simplify(u_eq);
%% Linearization
A = subs(jacobian(f,x(9:10)),[x(9:10);u],[x_eq;u_eq]);
B = subs(jacobian(f,u),[x(9:10);u],[x_eq;u_eq]);

A=simplify(A);
B=simplify(B);
return
variables=[u,dc,dw,BuildingHeight,Jp,Jm,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,g,mu,gearbox,min_length,x.'];
comments='Version: 1.0';
matlabFunction(f,'File','LiftDynamicsEquation.m','Vars',variables,'Comments',comments);


variables_x9=[u,dc,dw,BuildingHeight,Jp,Jm,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,g,mu,gearbox,min_length,x(9)];
matlabFunction(x_eq,u_eq,'File','LiftEquilibrium.m','Optimize',false,'Comments',comments);
matlabFunction(A,B,'File','LiftLinearSystem.m','Optimize',false,'Comments',comments);

vectorize_x('LiftDynamicsEquation.m')
