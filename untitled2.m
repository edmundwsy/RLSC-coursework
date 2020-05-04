function output = untitled2(t, input)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
m = 1.0; L = 0.5; I = 0.02; g = 9.8;
K =  [  -2.2361    2.2361   -5.8266   -1.7773    1.6541   -0.8562;
    2.2361    2.2361    5.8266    1.7773    1.6541    0.8562];
x0 = zeros(6, 1);
u0 = ones(2, 1) * m * g / 2;
u = -K * (input - x0) + u0;
output = equation(input, u);

end

