function output = equation(w, u)
%EQUATION non-linear dynamic equation of motion of planar quadrotor
%  $ \dot{x} = f(x, u)$
%  Input:
%       x: 6 by 1 vector
%           states:   [x, z, theta, x_dot, z_dot, theta_dot]
%
%       u: 2 by 1 vector
%           controls
%  Output:
%       6 by 1 vector
m = 1.0; L = 0.5; I = 0.02; g = 9.8;

x_ = sin(w(3)) * (u(1) + u(2)) / m;
z_ = cos(w(3)) * (u(1) + u(2)) / m - g;
theta_ = L/I * (u(2) - u(1)) - w(6) ^ 2;
output = [w(4); w(5); w(6); x_; z_; theta_];
end

