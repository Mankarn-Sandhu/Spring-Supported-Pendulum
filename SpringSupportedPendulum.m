function [t,x,pos]=lab1(r0,k)

% assume pendulum mass m=1
g=9.81; %gravity
m=1;
dt=0.01;        % integration time step
tspan=0:dt:500; % timespan of integration from 0 to 500
opt=odeset('Reltol',1e-9,'AbsTol',1e-9);
x0=input('enter initial condition in the form of [xinit0;vxinit0;yinit0;vyinit0] : ');
[t,x]=ode45(@pend,tspan,x0,opt,k,m,r0);
 pos(:,1)=x(:,1);
 pos(:,2)=-x(:,3);
 
function RHS=pend(t,x,k,m,r0)
r=sqrt(x(1)^2+x(3)^2);
dr=r-r0;
RHS(1,1)=x(2);
RHS(2,1)=-k/m*dr*x((1)/r);
RHS(3,1)=x(4);
RHS(4,1)=g-k/m*dr*x((3)/r);
end


end