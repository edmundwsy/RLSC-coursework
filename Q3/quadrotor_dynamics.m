function y = quadrotor_dynamics(x,u)

% === states and controls:
% x = [x z theta x_dot z_dot theta_dot]' = 
%       [x; y; quadrotor_angle; 
%       x_velocity; z_velocity; quadrotor_angle_velocity]
% u = [u1 u2]'     = [u1; u2]

% constants
h = 0.05;
m = 1.0;        % m = mass of the quadrotor
L = 0.5;        % L = length between two motors
I = 0.02;       % I = 
g = 9.8;        % g = gravity constant

% controls
u1 = u(1,:,:); % u1 = right motor
u2 = u(2,:,:); % u2 = left motor

o  = x(3,:,:); % o = quadrotor angle
w = x(6,:,:);
               % z = unit_vector(o)
z  = [sin(o); cos(o)]; 

do = L/I * (u2 - u1) - w.^2;  % do = change in theta_dot anglar acceleration
a = z.*sum(u) / m + [0; -g];
dy = h * [x(4:6, :, :); a; do];   % change in state
y  = x + dy;                % new state
