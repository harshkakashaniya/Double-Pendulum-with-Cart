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
t = 0:0.01:100;
r =10*ones(size(t)); %% Step input 
[y,t,x]=lsim(sys_cl,r,t,x0); %% simulation command
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot'); %% Plot of x and Theta 1
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum 1 angle (radians)')
 figure %% new figure
 
[AX,H1,H3] = plotyy(t,y(:,1),t,y(:,3),'plot');  %% Plot of x and Theta 2
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum 2 angle (radians)')
title('Step Response with LQR Control')