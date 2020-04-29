function PcPo =PcPo(A,B,C,D,g_p,g_o)
[~,n]=size(A);
disp(['P_c']);
P_c=ctrb(A,B)
disp(['P_o'])
P_o=obsv(A,C)
dimp=rank(P_c);
dimo=rank(P_o);
if dimp<n
    disp(['not Controllable'])
else 
    disp(['Controllable'])
end
if dimo<n
     disp(['not Obserberable'])
else 
    disp(['Obserberable'])
end
K=acker(A,B,g_p)
L=(acker(A',C',g_o))'   %g_p,g_o的元素元素个数应该等于n

