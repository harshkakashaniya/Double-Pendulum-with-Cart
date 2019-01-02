%% Part F - Luenberger Observer

clear all;
clear;

%%

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

% Change values here to for selecting output
C = [1 0 0 0 0 0;
     0 0 1 0 0 0
     0 0 0 0 1 0];
 
D = 0;

%% Open loop tracking

poles = [-1.61 -1.71 -1.6 -1.72 -1.51 -1.7]; % x, theta_1, theta_2
% poles = [-0.51 -0.55 -0.52 -0.50 -0.68 -0.6]; % x
% poles = [-0.3 -1.6 -1.7 -1.5 -1.52 -1.68]; % x theta 2
L = place(A', C', poles)';

states = {'x' 'x_dot' 'phi1' 'phi_dot1' 'phi2' 'phi_dot2'};
inputs = {'f'};
outputs = {'x'; 'phi1'; 'phi2'};

sys_cl = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.1:100;
r = ones(size(t));
x_initial = [2; 0; 0.5; 0; 0.5; 0];
x_hat = [2; 0; 0.5; 0; 0.5; 0];
x_est = x_hat';
iter = 2;
[y,t,x] = lsim(sys_cl,r,t,x_initial);

for i = 0.1:0.1:100
    x_hat = x_hat + A * x_hat + B.*r(iter) + L*(y(iter,:)' - C*x_hat);
    x_est = [x_est;x_hat'];
    iter = iter + 1;
end

t = t(15:end);
figure
subplot(3,1,1) 
plot(t,x(15:end,1), 'b')
hold on
plot(t,x_est(15:end,1),'r')
xlabel('t(sec)'),ylabel('Cart Position(m)'),legend('X','x-estimate')

subplot(3,1,2)
plot(t,x(15:end,3), 'b')
hold on
plot(t,x_est(15:end,3),'r')
xlabel('t(sec)'),ylabel('Pendulum Angle(phi1))'),legend('phi1','phi1-est')
% 
subplot(3,1,3)
plot(t,x(15:end,5), 'b')
hold on
plot(t,x_est(15:end,5),'r')
xlabel('t(sec)'),ylabel('Pendulum Angle(phi2))'),legend('phi2','phi2-est')

%% LQR with observer

Q = [100000     0     0     0     0     0;
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

% Place poles
poles = [-1.61 -1.71 -1.6 -1.72 -1 -1.7];
L = place(A', C', poles)';

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0;0];

states = {'x' 'x_dot' 'phi1' 'phi_dot1' 'phi2' 'phi_dot2' 'e1' 'e2' 'e3' 'e4' 'e5' 'e6'};
inputs = {'f'};
outputs = {'x'; 'phi1'; 'phi2'};

sys_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.1:100;
r = 1 * ones(size(t));
x_initial = [2, 0, 0.5, 0, 0.5, 0, 0, 0, 0, 0, 0, 0];

% Add x_intital to lsim as the last argument to simulate initial conditions. Else leave empty
[y,t,x] = lsim(sys_cl,r,t);
plot(t, y(:,1), 'r');
hold on
plot(t, y(:,2), 'g');
hold on
plot(t, y(:,3), 'b');
xlabel('t(sec)'),ylabel('Amplitude')
title('Case 3: x(t), theta1, theta2, step input');