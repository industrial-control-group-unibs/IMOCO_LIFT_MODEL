function [jerk,acc,vel,pos]=JerkMotionLaw(MaxJerk,t1,t2,t4,time)   

if (time<t1)
	jerk=MaxJerk;
	acc=MaxJerk*time;
	vel=(MaxJerk*time^2)/2;
	pos=(MaxJerk*time^3)/6;
elseif (time<t1 + t2)
	jerk=0;
	acc=MaxJerk*t1;
	vel=(MaxJerk*t1^2)/2 - MaxJerk*t1*(t1 - time);
	pos=(MaxJerk*t1^3)/6 - (MaxJerk*t1*time*(t1 - time))/2;
elseif (time<2*t1 + t2)
	jerk=-MaxJerk;
	acc=MaxJerk*t1 + MaxJerk*(t1 + t2 - time);
	vel=(MaxJerk*t1*(t1 + 2*t2))/2 - (MaxJerk*(4*t1*t2 - 4*t1*time - 2*t2*time + 3*t1^2 + t2^2 + time^2))/2;
	pos=(MaxJerk*t1^3)/6 - (MaxJerk*(t1 + t2 - time)*(t1*t2 + 5*t1*time + 2*t2*time - t1^2 - t2^2 - time^2))/6 + (MaxJerk*t1*t2*(t1 + t2))/2;
elseif (time<2*t1 + t2 + t4)
	jerk=0;
	acc=0;
	vel=MaxJerk*t1*(t1 + t2);
	pos=(MaxJerk*t1*(3*t1*t2 + 2*t1^2 + t2^2))/2 - MaxJerk*t1*(t1 + t2)*(2*t1 + t2 - time);
elseif (time<3*t1 + t2 + t4)
	jerk=-MaxJerk;
	acc=MaxJerk*(2*t1 + t2 + t4 - time);
	vel=MaxJerk*t1*(t1 + t2) - (MaxJerk*(2*t1 + t2 + t4 - time)^2)/2;
	pos=(MaxJerk*t1*(t1 + t2)*(2*t1 + t2 + 2*t4))/2 - (MaxJerk*(2*t1 + t2 + t4 - time)*(2*t1*t2 - 4*t1*t4 - 2*t2*t4 + 4*t1*time + 2*t2*time + 2*t4*time + 2*t1^2 - t2^2 - t4^2 - time^2))/6;
elseif (time<3*t1 + 2*t2 + t4)
	jerk=0;
	acc=-MaxJerk*t1;
	vel=MaxJerk*t1*(3*t1 + t2 + t4 - time) + (MaxJerk*t1*(t1 + 2*t2))/2;
	pos=(MaxJerk*t1*(15*t1*t2 + 6*t1*t4 + 6*t2*t4 + 11*t1^2 + 3*t2^2))/6 - (MaxJerk*t1*(13*t1*t2 + 7*t1*t4 + 4*t2*t4 - 7*t1*time - 4*t2*time - 2*t4*time + 12*t1^2 + 3*t2^2 + t4^2 + time^2))/2;
elseif (time<4*t1 + 2*t2 + t4)
	jerk=MaxJerk;
	acc=- MaxJerk*t1 - MaxJerk*(3*t1 + 2*t2 + t4 - time);
	vel=(MaxJerk*t1^2)/2 + (MaxJerk*(16*t1*t2 + 8*t1*t4 + 4*t2*t4 - 8*t1*time - 4*t2*time - 2*t4*time + 15*t1^2 + 4*t2^2 + t4^2 + time^2))/2;
	pos=(MaxJerk*t1*(18*t1*t2 + 6*t1*t4 + 6*t2*t4 + 11*t1^2 + 6*t2^2))/6 - (MaxJerk*(48*t1*t2^2 + 96*t1^2*t2 + 12*t1*t4^2 + 48*t1^2*t4 + 6*t2*t4^2 + 12*t2^2*t4 + 12*t1*time^2 - 48*t1^2*time + 6*t2*time^2 - 12*t2^2*time + 3*t4*time^2 - 3*t4^2*time + 63*t1^3 + 8*t2^3 + t4^3 - time^3 + 48*t1*t2*t4 - 48*t1*t2*time - 24*t1*t4*time - 12*t2*t4*time))/6;
else
	jerk=0;
	acc=0;
	vel=0;
	pos=MaxJerk*t1*(t1 + t2)*(2*t1 + t2 + t4);
end