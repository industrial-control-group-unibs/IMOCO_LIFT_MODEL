clear all;close all;clc;
MaxAcc=2;
MaxJerk=2;
MaxVel=2;

distance=3;

if MaxVel>MaxAcc^2/(2*MaxJerk)
    %case 1
    limit_distance_case1=(MaxVel*(MaxAcc^2 + MaxVel*MaxJerk))/(MaxAcc*MaxJerk);
    

    if (distance>limit_distance_case1)
        % case 1.1
        CruiseVel=MaxVel;
        law_case=11;
    else
        CruiseVel=-(MaxAcc^2 - MaxAcc^(1/2)*(MaxAcc^3 + 4*distance*MaxJerk^2)^(1/2))/(2*MaxJerk);
        
        if CruiseVel>MaxAcc^2/(2*MaxJerk)
            %case 1.2
            law_case=12;
        else
            limit_distance_case2=(2*CruiseVel^(3/2))/MaxJerk^(1/2);
            if distance>limit_distance_case2
                law_case=21;
            else
                law_case=22;
            end
        end
    end

else
    limit_distance_case2=(2*MaxVel^(3/2))/MaxJerk^(1/2);
    if distance>limit_distance_case2
        law_case=21;
    else
        law_case=22;
    end
end

switch law_case
    case 11
        t1=MaxAcc/MaxJerk;
        t2=(- MaxAcc^2/MaxJerk + MaxVel)/MaxAcc;
        t4=(distance - (MaxVel*(MaxAcc^2 + MaxVel*MaxJerk))/(MaxAcc*MaxJerk))/MaxVel;
        CruiseVel=MaxVel;
    case 12
        t1=MaxAcc/MaxJerk;
        CruiseVel=-(MaxAcc^2 - MaxAcc^(1/2)*(MaxAcc^3 + 4*distance*MaxJerk^2)^(1/2))/(2*MaxJerk);
        t2=-(3*MaxAcc^(3/2) - (MaxAcc^3 + 4*distance*MaxJerk^2)^(1/2))/(2*MaxAcc^(1/2)*MaxJerk);
        t4=0;
    case 21
        %CruiseVel=MaxVel;
        t1=CruiseVel^(1/2)/MaxJerk^(1/2);
        t2=0;
        t4=(distance - (2*CruiseVel^(3/2))/MaxJerk^(1/2))/CruiseVel;
        
    otherwise % case 22
        CruiseVel=((MaxJerk^(1/2)*distance)/2)^(2/3);
        t1=CruiseVel^(1/2)/MaxJerk^(1/2);
        t2=0;
        t4=0;
end
%%
Ts=1e-2;

t=(0:Ts:(4*t1+2*t2+t4))'; % t1+t2+t3+t4+t5+t6+t7, t1=t3=t5=t6,t2=t6


for idx=1:length(t)
    [jerk(idx,1),acc(idx,1),vel(idx,1),pos(idx,1)]=JerkMotionLaw(MaxJerk,t1,t2,t4,t(idx,1));
end

subplot(4,1,1)
plot(t,pos)
subplot(4,1,2)
plot(t,vel)
subplot(4,1,3)
plot(t,acc)
subplot(4,1,4)
plot(t,jerk)


