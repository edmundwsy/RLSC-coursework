% planar quadrotor LQR control
g=9.81; % gravity
m=1.4;  % total mass of quadrotor
Ixx=m/12 * (0.35^2 + 0.1^2); % moment of inertia about X-axis

T = 0:0.01:5; % Simulation Time 
U = zeros(2,length(T)); % Control input (feedforward)

X0 = [-1,0,0,0,0,0.0]; % Initial State

A = [0 0  0 1 0 0; 
     0 0  0 0 1 0; 
     0 0  0 0 0 1; 
     0 0 -g 0 0 0; 
     0 0  0 0 0 0; 
     0 0  0 0 0 0];
 
B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/Ixx];

C = eye(6);
D = 0;

Q = eye(6); 
R = eye(2);

K = lqr(A,B,Q,R);

%sys=ss(A,B,C,D); % open-loop system
sys=ss(A-B*K,B,C,D); % closed-loop system

% run the simulation
lsim(sys,U,T,X0);

% to recover the (feedback) control inputs, run the following:
%X=lsim(sys,U,T,X0);
%U= -K*X';


