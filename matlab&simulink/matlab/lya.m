function lya=lya(A,B,C,D,init_x,time)
%A,B,C,D为系数矩阵,init_x为x向量初始状态,time为所经历的时间,输出为P,并绘制出图像.
Q=eye(size(A));
P=lyap(A,Q)
sys=ss(A,B,C,D);
[~,~,x]=initial(sys,init_x,time);
%通过判断P是否正定确定是否稳定.

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
    disp("系统稳定")
    plot(x(:,1),x(:,2))
    grid on
else 
    disp("系统不稳定")
end


