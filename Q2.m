clc
clear all


Q = diag([0,0,1,1,1,1]);
R = diag([1,1]);


% dimension constants
L = 0.5;
state_dim = 6;
control_dim = 2;
total_dim = state_dim + control_dim;

% state s = [x, z, thata, x_d, z_d, theta_d]
% input u = [u1, u2]
% x = [x, u] for all Nodes

% optimisation constants
h = 0.1; 
t0 = 0; 
tf = 2.0;
nNodes = ceil((tf - t0 ) / h) + 1;

%trapezoidal integration
x0 = zeros(total_dim, nNodes);
x_ic = [0,0,0,0,0,0];
x_fc = [2,1,0,0,0,0];

% cost function
costFun = @(x) sumsqr(x(3:8, :));

% constraints and boundries
Aeq=[];
beq=[]; 
A=[];
b=[]; 
lb = [];
ub = [];
nonlcon = @dynamicConstraintQuadrotorSystem;

% solving the optimisation problem
options = optimoptions(@fmincon, 'Algorithm', 'sqp','MaxIterations',100);
x = fmincon(costFun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);


% trapezoidal interpolation
collocation_position = x(1:3, :);
collocation_velocity = x(4:6, :);
collocation_force = x(7:8, :);
collocation_time = (0:nNodes -1) * h;





%% animation

figure('Name','Quadrotor Motion Animation','NumberTitle','off');
plot([x_fc(1), 0] ,[x_fc(2), 0], '*')
axis([-0.7,  x_fc(1)+0.7, -0.7, x_fc(2)+0.7])

hold on
axis equal

plot(collocation_position(1,:), collocation_position(2,:), 'k-')

% quadrotor = rectangle('EdgeColor','k','FaceColor','r');
dt_int=0.01;

nInterpolation=( nNodes - 1 ) * h/dt_int + 1;

interpolation_position=zeros(3, nInterpolation);
interpolation_velocity=zeros(3, nInterpolation);
interpolation_force=zeros(2, nInterpolation);
interpolation_time=(0:nInterpolation-1)*h*0.1;

array_index=1;

boolPlot=0;

for i = 1:(nNodes-1)
    for j=0:dt_int:(h)
        interpolation_force(:, array_index)=collocation_force(:, i) + ...
            (j/h)*(collocation_force(:, i+1)-collocation_force(:, i));
        interpolation_velocity(:, array_index)=collocation_velocity(:, i) + ...
            (j/h)*(collocation_velocity(:, i+1)-collocation_velocity(:, i));
        interpolation_position(:, array_index)=collocation_position(:, i) + ...
            collocation_velocity(:, i)*j+(0.5*(j^2)/h)*(collocation_velocity(:, i+1)-collocation_velocity(:, i));
        
        left_edge = [interpolation_position(1, array_index) - ...
            0.5 * L * cos(interpolation_position(3, array_index)), ...
            interpolation_position(2, array_index) + ...
            0.5 * L * sin(interpolation_position(3, array_index))];
        right_edge = [interpolation_position(1, array_index) + ...
            0.5 * L * cos(interpolation_position(3, array_index)), ...
            interpolation_position(2, array_index) - ...
            0.5 * L * sin(interpolation_position(3, array_index))];
        force1 = [interpolation_force(1, array_index) * sin(interpolation_position(3, array_index)), ...
            interpolation_force(1, array_index) * cos(interpolation_position(3, array_index))];
        force2 = [interpolation_force(2, array_index) * sin(interpolation_position(3, array_index)), ...
            interpolation_force(2, array_index) * cos(interpolation_position(3, array_index))];
                
        quadrotor = line([left_edge(1), right_edge(1)], ...
            [left_edge(2), right_edge(2)], ...
            'Color', 'red', 'LineWidth', 3.0, 'Marker', 'o');
        arrow1 = quiver(left_edge(1), left_edge(2), force2(1), force2(2), 0.1, ...
            'Color', 'blue');
        arrow2 = quiver(right_edge(1), right_edge(2), force1(1), force1(2), 0.1, ...
            'Color', 'blue');        

        if(boolPlot==1)
            drawnow
        end
        
        pause(3 * dt_int);
        if(j~=h)
            array_index=(array_index)+1;
        end
        
        if (array_index ~= nInterpolation)
            delete(quadrotor)
            delete(arrow1)
            delete(arrow2)
        end
    end
    
end

figure('Name','Quadrotor Motion Interpolation Graph','NumberTitle','off');
ax4=subplot(3,1,1);
plot(ax4, interpolation_time, interpolation_position)
xlabel('time');
ylabel('position (m)');
ax5=subplot(3,1,2);
plot(ax5, interpolation_time, interpolation_velocity)
xlabel('time');
ylabel('velocity (m/s)');
ax6=subplot(3,1,3);
plot(ax6, interpolation_time, interpolation_force)
xlabel('time');
ylabel('force (N)');
