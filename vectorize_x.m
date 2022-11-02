function vectorize_x(filename)
txt=readlines(filename);
txt(1)=replace(txt(1),"state1,state2,state3,state4,state5,state6,state7,state8,state9,state10","state");
for idx=9:length(txt)
    for istate=10:-1:1
        txt(idx)=replace(txt(idx),"state"+istate,"state("+istate+")");
    end
    disp(txt(idx))
end

textbody=txt(1)+newline;
for idx=5:length(txt)
    textbody=textbody+txt(idx)+newline;
end
fid=fopen(filename,"w") 
fprintf(fid,"%s",textbody);
fclose(fid)