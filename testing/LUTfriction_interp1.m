function friction=LUTfriction_interp1(position,velocity,LUT_position,LUT_Coloumb_friction,LUT_viscous_friction)

Coloumb_friction=interp1(LUT_position,LUT_Coloumb_friction,position);
viscous_friction=interp1(LUT_position,LUT_viscous_friction,position);

direction=tanh(velocity*100.0);  % approximation of sign

friction=Coloumb_friction*direction+viscous_friction*velocity;
