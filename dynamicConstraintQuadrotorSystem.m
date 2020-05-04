function [c, ceq] = dynamicConstraintQuadrotorSystem(x)

% define constants
m = 1.0; L = 0.5; I = 0.02; g = 9.8;
tf = 2.0;
h = 0.1;
nNodes = tf / h + 1;

% states
state_dim = 6;
x_ic = [0;0;0;0;0;0];
x_N = [2;1;0;0;0;0];

nConstraint = state_dim * (nNodes - 1); % 2n + 4
c= [];

ceq = zeros(nConstraint + 2 * state_dim, 1);
ceq_ = zeros(state_dim, nNodes - 1);

for i = 1:(nNodes-1)
    ceq_(1, i) = x(1, i) - x(1, i+1) + 0.5 * h *(x(4, i) + x(4, i+1));
    ceq_(2, i) = x(2, i) - x(2, i+1) + 0.5 * h *(x(5, i) + x(5, i+1));
    ceq_(3, i) = x(3, i) - x(3, i+1) + 0.5 * h *(x(6, i) + x(6, i+1));
    ceq_(4, i) = x(4, i) - x(4, i+1) + 0.5 * h *(sin(x(3, i)) * (x(7, i) + x(8, i)) + sin(x(3, i+1)) * (x(7, i+1) + x(8, i+1)))/m ;
    ceq_(5, i) = x(5, i) - x(5, i+1) + 0.5 * h *(cos(x(3, i)) * (x(7, i) + x(8, i)) + cos(x(3, i+1)) * (x(7, i+1) + x(8, i+1)) - 2*m*g)/m ;
    ceq_(6, i) = x(6, i) - x(6, i+1) + 0.5 * h *(L / I * (x(8, i) - x(7, i)) - x(6, i) ^ 2 + L / I * (x(8, i+1) - x(7, i+1)) - x(6, i+1) ^ 2);
end

ceq(1:nConstraint) = reshape(ceq_, [nConstraint, 1]);
ceq(nConstraint + 1: nConstraint + state_dim) = x(1:6, nNodes) - x_N;
ceq(nConstraint + state_dim + 1: nConstraint + 2 * state_dim) = x(1:6, 1) - x_ic;

end
