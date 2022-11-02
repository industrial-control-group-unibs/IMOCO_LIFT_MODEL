clear all;close all;clc

syms MaxJerk time t1 t2 t4 last_acc last_vel last_pos
last_acc=0;
last_vel=0;
last_pos=0;

t=[t1 t2 t1 t4 t1 t2 t1];
cumt=[0 cumsum(t)];
J=MaxJerk*[1  0 -1 0 -1 0 1 0];
for idx=1:length(cumt)
    jerk=J(idx);
    acc=last_acc+simplify(int(jerk,time,cumt(idx),time));
    vel=last_vel+simplify(int(acc ,time,cumt(idx),time));
    pos=last_pos+simplify(int(vel ,time,cumt(idx),time));

    if (idx<length(cumt))
        last_acc=simplify(subs(acc,time,cumt(idx+1)));
        last_vel=simplify(subs(vel,time,cumt(idx+1)));
        last_pos=simplify(subs(pos,time,cumt(idx+1)));
        if idx==1
            fprintf('if (time<%s)\n',char(cumt(idx+1)))
        else
            fprintf('elseif (time<%s)\n',char(cumt(idx+1)))
        end
    else
        fprintf('else\n')
    end

    fprintf('\tjerk=%s;\n',char(jerk));
    fprintf('\tacc=%s;\n',char(acc));
    fprintf('\tvel=%s;\n',char(vel));
    fprintf('\tpos=%s;\n',char(pos));

    A(idx,1)=acc;
    V(idx,1)=vel;
    P(idx,1)=pos;

    
end

fprintf('end\n')


