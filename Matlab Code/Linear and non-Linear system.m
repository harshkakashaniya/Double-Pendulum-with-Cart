clear all;
clear;
format long
syms F m1 g th1 dth1 l1 m2 th2 dth2 l2 M
%%---------------------------------------------------------------
Z=(F- m1*g*cos(th1)*sin(th1)- m1*l1*((dth1)^2)*sin(th1)-m2*g*cos(th2)*sin(th2) - m2*l2*(dth2^2)*sin(th2))/(M + m1*sin(th1)^2 + m2*sin(th2)^2);
th2doubledot = (Z*cos(th2)-g*sin(th2)) / l2;
th1doubledot = (Z*cos(th1) - g*sin(th1)) / l1;
%%---------------------------------------------------------------
A=simplify(diff(Z,th1));
B=simplify(diff(Z,dth1));
C=simplify(diff(Z,th2));
D=simplify(diff(Z,dth2));
%%---------------------------------------------------------------
I = simplify(diff(th2doubledot,th1));
J = simplify(diff(th2doubledot,dth1));
K = simplify(diff(th2doubledot,th2));
L = simplify(diff(th2doubledot,dth2));
%%---------------------------------------------------------------
E = simplify(diff(th1doubledot, th1));
F = simplify(diff(th1doubledot, dth1));
G = simplify(diff(th1doubledot, th2));
H = simplify(diff(th1doubledot, dth2));
%%---------------------------------------------------------------
Jacob=[0 1 0 0 0 0;
   0 0 A B C D;
   0 0 0 1 0 0;
   0 0 E F G H;
   0 0 0 0 0 1;
   0 0 I J K L]; %% Jacobian system
%%
A1=subs(A,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
B1=subs(B,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
C1=subs(C,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
D1=subs(D,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
E1=subs(E,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
F1=subs(F,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
G1=subs(G,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
H1=subs(H,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
I1=subs(I,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
J1=subs(J,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
K1=subs(K,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);
L1=subs(L,[th1 th2 dth1 dth2 ],[0 0 0 0 ]);  %% Jacobian system about eqbm point

%%---------------------------------------------------------------
Jacob1=[0 1 0 0 0 0;
       0 0 A1 B1 C1 D1;
       0 0 0 1 0 0;
       0 0 E1 F1 G1 H1;
       0 0 0 0 0 1;
       0 0 I1 J1 K1 L1]

%
A1=subs(A,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
B1=subs(B,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
C1=subs(C,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
D1=subs(D,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
%---------------------------------------------------------------
E1=subs(E,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
F1=subs(F,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
G1=subs(G,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
H1=subs(H,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
%---------------------------------------------------------------
I1=subs(I,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
J1=subs(J,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
K1=subs(K,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
L1=subs(L,[th1 th2 dth1 dth2 M m1 m2 l1 l2 g],[0 0 0 0 1000 100 100 20 10 9.81]);
%---------------------------------------------------------------
Jacob_value=[0 1 0 0 0 0;
       0 0 A1 B1 C1 D1;
       0 0 0 1 0 0;
       0 0 E1 F1 G1 H1;
       0 0 0 0 0 1;
       0 0 I1 J1 K1 L1] %% construction of final A matrix 
%---------------------------------------------------------------

B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)]; %% final B matrix
Bmatrix = subs(B,[M m1 m2 l1 l2 g],[1000 100 100 20 10 9.81]);
ctrb= [Bmatrix Jacob_value*Bmatrix Jacob_value^2*Bmatrix Jacob_value^3*Bmatrix Jacob_value^4*Bmatrix Jacob_value^5*Bmatrix];  %% Controllability
rank(ctrb) %% rank of A
det(ctrb) %% determint of A