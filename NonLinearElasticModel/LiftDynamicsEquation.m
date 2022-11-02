function f = LiftDynamicsEquation(u,dc,dw,BuildingHeight,Jp,Jm,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,g,mu,gearbox,min_length,state)
%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    02-Nov-2022 11:17:48

%Version: 1.0
t2 = Rp.^2;
t3 = gearbox.^2;
t4 = min_length.^2;
t5 = state(9).^2;
t6 = LinearStiffness.*min_length.*2.0;
t7 = Rp.*gearbox.*state(9);
t8 = 1.0./mu;
t9 = BuildingHeight.*LinearStiffness.*2.0;
t10 = min_length./2.0;
t12 = (gearbox.*state(10))./2.0;
t11 = g.*mu.*t4;
t13 = min_length+t7;
t14 = t7./2.0;
t16 = g.*mu.*t2.*t3.*t5;
t15 = 1.0./t13;
t17 = t10+t14;
t18 = 1.0./t17;
mt1 = [state(2);-(dw+Mw.*g+LinearDamping.*t18.*(state(2)-state(4)+t12)+LinearStiffness.*t18.*(state(1)-state(3)+t17))./Mw;state(4);-t8.*t15.^2.*(-t6-t9+t11+t16-LinearDamping.*state(2).*2.0+LinearDamping.*state(4).*4.0-LinearStiffness.*state(1).*2.0+LinearStiffness.*state(3).*4.0+g.*min_length.*mu.*t7.*2.0);state(6);(t8.*t15.*(t6+t9-t11+t16-LinearDamping.*state(6).*4.0+LinearDamping.*state(8).*2.0-LinearStiffness.*state(5).*4.0+LinearStiffness.*state(7).*2.0-BuildingHeight.*g.*min_length.*mu-BuildingHeight.*g.*mu.*t7))./(BuildingHeight+min_length-t7);state(8);-(dc+Mc.*g-LinearDamping.*t18.*(state(6)-state(8)+t12)+LinearStiffness.*t18.*(BuildingHeight./2.0-state(5)+state(7)+t10-t14))./Mc;state(10)];
mt2 = [-(t15.*(-min_length.*u-t7.*u+MotorViscousFriction.*min_length.*state(10)+MotorViscousFriction.*state(10).*t7+LinearStiffness.*state(9).*t2.*t3.*2.0-BuildingHeight.*LinearStiffness.*Rp.*gearbox+LinearDamping.*Rp.*gearbox.*state(4).*2.0-LinearDamping.*Rp.*gearbox.*state(6).*2.0+LinearStiffness.*Rp.*gearbox.*state(3).*2.0-LinearStiffness.*Rp.*gearbox.*state(5).*2.0+LinearDamping.*Rp.*state(10).*t3.*2.0))./(Jm+Jp.*t3)];
f = [mt1;mt2];
