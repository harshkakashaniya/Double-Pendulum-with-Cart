clear all
clear
A =[ 0, 1,            0, 0,            0, 0;
 0, 0,    -981/1000, 0,    -981/1000, 0;
 0, 0,            0, 1,            0, 0;
 0, 0, -10791/20000, 0,   -981/20000, 0;
 0, 0,            0, 0,            0, 1;
 0, 0,   -981/10000, 0, -10791/10000, 0];

B =[    0;
    1/1000;
         0;
   1/20000;
         0;
   1/10000];

C = [1 0 0 0 0 0;
     0 0 1 0 0 0
     0 0 0 0 1 0];
 
D = 0;
states = {'x' 'x_dot' 'phi1' 'phi_dot1' 'phi2' 'phi_dot2'};
inputs = {'f'};
outputs = {'x'; 'phi1';'phi2'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A) %% poles without Control

co = ctrb(sys_ss);
controllability = rank(co)

Q =[100000     0     0     0     0     0;
    0    10     0     0     0     0;
    0     0     1000000     0     0     0;
    0     0     0    10     0     0;
    0     0     0     0     1000000     0;
    0     0     0     0     0    10] %cost matrix 


R=0.001

K = lqr(A,B,Q,R)
% K=[0 0 0 0 0 0 ];
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

poles1 = eig(Ac); %% poles with LQR
states = {'x' 'x_dot' 'phi1' 'phi_dot1' 'phi2' 'phi_dot2'};
inputs = {'f'};
outputs = {'x'; 'phi1'; 'phi2'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% hold on
x0=[2,0,0.3,0,0.3,0]  %% initial condition
t = 0:0.1:100;
r =8*ones(size(t)); %% Step input 
[y,t,x]=lsim(sys_cl,r,t,x0); %% simulation command
% figure
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot'); %% Plot of x and Theta 1
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum 1 angle (radians)')
%  figure %% new figure
%  
% [AX,H1,H3] = plotyy(t,y(:,1),t,y(:,3),'plot');  %% Plot of x and Theta 2
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum 2 angle (radians)')
% title('Step Response with LQR Control')
%%
pend1x=-5*sin(x(:,3))+x(:,1);
pend1y=-5*cos(x(:,3));
pend2x=-10*sin(x(:,5))+x(:,1);
pend2y=-10*cos(x(:,5));
hold on;
axis([-5 5 -15 5]);
axis('square');
grid on;
length= length(t);
hbead=line(x(1,1),t(1),'color','r','marker','square','markersize',30);

line1x=[x(:,1),pend1x];
line1y=[zeros(length,1),pend1y];
hbead1=line(line1x(1),line1y(1),'marker','hexagram','markersize',5,'color','g');

line2x=[x(:,1),pend2x];
line2y=[zeros(length,1),pend2y];
hbead2=line(line2x(1),line2y(1),'marker','hexagram','markersize',5);

for k=2:length
set(hbead,'xdata',x(k,1),'ydata',0);
set(hbead1,'xdata',line1x(k,:),'ydata',line1y(k,:));
set(hbead2,'xdata',line2x(k,:),'ydata',line2y(k,:));
drawnow
% F(k)=getframe(gcf);
end

% video=VideoWriter('123.avi','Uncompressed AVI');
% open(video);
% writeVideo(video,F);
% close video