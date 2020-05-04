function cost = quadrotor_cost(x, u)
% cost function for car-parking problem


final = isnan(u(1,:));
u(:,final)  = 0;

R = 0.1 * eye(2);
Q = 1000 * eye(6);
xd = [2.0; 1.0; 0; 0; 0; 0];

% control cost
lu = sum(R * u.^2); 
lx = sum(Q * (x - xd).^2);

% total cost
cost = lu + lx;





