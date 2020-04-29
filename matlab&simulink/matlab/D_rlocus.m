function D_rlocus=D_rlocus(nums,dens)

sys=tf(nums,dens);
rlocus(sys);
hold on
x=[-1:0.01:1];
y=sqrt(1-x.^2); %draw the unit circle
plot(x,y,'--',x,-y,'--')
%disp("To find the K of your choosen point")
%rlocfind(sys)