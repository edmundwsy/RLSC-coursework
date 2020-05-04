clc
clear all
%% solution for Part 1 question 1~2
% Initialise
m = 1.0; L = 0.5; I = 0.02; g = 9.8;
x0 = zeros(6, 1);
u0 = ones(2, 1) * m * g / 2;
A = [0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1;
    0, 0, g, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0];

B = [0, 0;
    0, 0;
    0, 0;
    0, 0;
    1/m, 1/m;
    -L/I, L/I];

Q = diag([10, 10, 10, 1, 1, 1]);
R = diag([1, 1]);

K = lqr(A, B, Q, R)


%% solution for Part 1 question 3 
% Euler Forward 
T = 100; h = 0.05;
x = zeros(6, T);
u = zeros(2, T);

x = [];
x_init = [0.06; -0.07; 1.395; 0.3; -0.5; 0.0];
% x_init = [0.06; -0.07; -pi/2; 0.1; -0.5; -pi/2];
x(:,1) = x_init;


% 4th order R-K method

for k = 1:1:100
    u(:, k) = -K * (x(:, k) - x0) + u0;
    
    K1 = h * equation(x(:, k), u(:, k));
    K2 = h * equation(x(:, k) + 0.5 * K1, u(:, k) + 0.5 * h);
    K3 = h * equation(x(:, k) + 0.5 * K2, u(:, k) + 0.5 * h);
    K4 = h * equation(x(:, k) + K3, u(:, k) + h);
    x(:, k + 1) = x(:, k) + ( K1 + 2 * K2 + 2 * K3 + K4) / 6;
end

% plot
figure()
labels = ["x", "z", "$\theta$", "$\dot{x}$", "$\dot{z}$", "$\dot{\theta}$"];
for i = 1:6
    subplot(6, 1, i)
    plot(x(i,:))
    yl = ylabel(labels(i));
    set(yl, 'Interpreter', 'latex')
    set(yl, 'FontSize', 17)
end
xlabel("Time")
sgtitle("States")
% ylabel("Amplitude")


%% solution for Q4: find region of attraction

num = 200;
u_ = linspace(-pi,pi, num);
v = linspace(-3,3, num);
threshold = 1000;

flag = ones(num, num);

% check every coordinates of parameters
for i = 1:num
    
    for j = 1:num
        x = zeros(6, 100);
        u = zeros(2, 100);
        x(:,1) = x_init;    
        x(3,1) = u_(i);
        x(6,1) = v(j);
        for k = 1:1:100
            u(:, k) = -K * (x(:, k)) + u0;

            K1 = h * equation(x(:, k), u(:, k));
            K2 = h * equation(x(:, k) + 0.5 * K1, u(:, k) + 0.5 * h);
            K3 = h * equation(x(:, k) + 0.5 * K2, u(:, k) + 0.5 * h);
            K4 = h * equation(x(:, k) + K3, u(:, k) + h);
            x(:, k + 1) = x(:, k) + ( K1 + 2 * K2 + 2 * K3 + K4) / 6;
            if max(abs(x(:, k+1))) > threshold
                flag(i, j) = 0;
            end
        end
    end   
   
end

figure(2)
mesh(u_, v, flag)
xlabel('theta')
ylabel('theta dot')
view([0, 90])

% imshow(flag)

