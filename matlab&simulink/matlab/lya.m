function lya=lya(A,B,C,D,init_x,time)
%A,B,C,DΪϵ������,init_xΪx������ʼ״̬,timeΪ��������ʱ��,���ΪP,�����Ƴ�ͼ��.
Q=eye(size(A));
P=lyap(A,Q)
sys=ss(A,B,C,D);
[~,~,x]=initial(sys,init_x,time);
%ͨ���ж�P�Ƿ�����ȷ���Ƿ��ȶ�.

p_eig=eig(P);
p_size=size(p_eig);
num=p_size(1)*p_size(2);
flag=0;
for i =1:num
    if(p_eig(i)<0)
        flag=1;
    end
end
if(flag==0)
    disp("ϵͳ�ȶ�")
    plot(x(:,1),x(:,2))
    grid on
else 
    disp("ϵͳ���ȶ�")
end


