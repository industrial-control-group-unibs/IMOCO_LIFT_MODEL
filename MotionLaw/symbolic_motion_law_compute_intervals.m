clear all;close all;clc;

syms MaxAcc MaxVel MaxJerk time distance positive;
syms t1 t2 t4 positive

t3=t1;

jerk_t1=MaxJerk;
acc_t1=int(jerk_t1,time,0,time);
vel_t1=int(acc_t1,time,0,time);
pos_t1=int(vel_t1,time,0,time);

acc_end_of_t1=subs(acc_t1,time,t1);
vel_end_of_t1=subs(vel_t1,time,t1);
pos_end_of_t1=subs(pos_t1,time,t1);
jerk_t2=0;
acc_t2=acc_end_of_t1+int(jerk_t2,time,t1,time);
vel_t2=vel_end_of_t1+int(acc_t2,time,t1,time);
pos_t2=pos_end_of_t1+int(vel_t2,time,t1,time);


acc_end_of_t2=subs(acc_t2,time,t1+t2);
vel_end_of_t2=subs(vel_t2,time,t1+t2);
pos_end_of_t2=subs(pos_t2,time,t1+t2);

jerk_t3=-MaxJerk;
acc_t3=acc_end_of_t2+int(jerk_t3,time,t1+t2,time);
vel_t3=vel_end_of_t2+int(acc_t3,time,t1+t2,time);
pos_t3=pos_end_of_t2+int(vel_t3,time,t1+t2,time);

acc_end_of_t3=subs(acc_t3,time,t1+t2+t3);
vel_end_of_t3=subs(vel_t3,time,t1+t2+t3);
pos_end_of_t3=subs(pos_t3,time,t1+t2+t3);

%% case 1 vel_end_of_t1<MaxSpeed

t1_case1=MaxAcc/MaxJerk;

t2_case1=solve(subs(vel_end_of_t3,t1,t1_case1)==MaxVel,t2);

limit_distance_case11=2*simplify(subs(pos_end_of_t3,[t1,t2],[t1_case1,t2_case1]));

% CASE 1.1: distance > limit_distance
t4_case11=simplify(solve(t4*MaxVel+limit_distance_case11==distance,t4));
cruise_speed_case11=MaxVel;
% CASE 1.2: distance <limit_distance, change MaxVel
limit_distance_case12=2*simplify(subs(pos_end_of_t3,[t1],[t1_case1]));

t4_case12=0;
t2_case12=simplify(solve(limit_distance_case12==distance,t2));
cruise_speed_case12=simplify(subs(vel_end_of_t3,[t1,t2],[t1_case1,t2_case12]))


%% case 2 vel_end_of_t1>MaxSpeed
t2_case2=0;

t1_case2=solve(subs(vel_end_of_t3,t2,t2_case2)==MaxVel,t1);

limit_distance_case2=2*simplify(subs(pos_end_of_t3,[t1,t2],[t1_case2,t2_case2]));


% CASE 2.1: distance > limit_distance
t4_case21=simplify(solve(t4*MaxVel+limit_distance_case2==distance,t4));
cruise_speed_case21=MaxVel;
% CASE 2.2: distance <limit_distance, change MaxVel
t4_case22=0;
cruise_speed_case22=solve(limit_distance_case2==distance,MaxVel)