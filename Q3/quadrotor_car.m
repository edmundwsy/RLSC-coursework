function quadrotor_car
% A demo of iLQG/DDP with quadrotor-parking dynamics
clc;
close all

fprintf(['\nA demonstration of the iLQG algorithm '...
'with quadrotor parking dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(x,u,i) quadrotor_dyn_cst(x,u,full_DDP);
T       = 80;              % horizon
x0      = [0;0;0;0;0;0];   % initial state
u0      = zeros(2,T);       % initial controls
Op.maxIter = 1000;          % max iteration
Op.tolGrad = 0.01;
Op.tolFnc = 0.01;

% prepare the visualization window and graphics callback
figure(9);
set(gcf,'name','quadrotor','Menu','none','NumberT','off')
set(gca,'xlim',[-0.5 2.5],'ylim',[-1 2],'DataAspectRatio',[1 1 1])
grid on
box on

% plot target configuration with light colors
handles = quadrotor_plot([0 0 0 0]', [5 5]');
fcolor  = get(handles,'facecolor');
ecolor  = get(handles,'edgecolor');
fcolor  = cellfun(@(x) (x+3)/4,fcolor,'UniformOutput',false);
ecolor  = cellfun(@(x) (x+3)/4,ecolor,'UniformOutput',false);
set(handles, {'facecolor','edgecolor'}, [fcolor ecolor])

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','b','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

% === run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);

% animate the resulting trajectory
figure(9)
handles = [];
for i=1:T
   set(0,'currentfigure',9);
   delete(handles)
   handles = quadrotor_plot(x(:,i), u(:,i));
   drawnow   
   pause(0.05);
end








% ======== graphics functions ========
function h = quadrotor_plot(x,u)

body        = [0.5 0.05 0];           % body = [width length curvature]
bodycolor   = [1.0 0 0];
force1  = [0.05 * u(1) 0.01 0 body(1)]; % force1 [width length curvature x]
force2  = [0.05 * u(2) 0.01 0 body(1)]; % force1 [width length curvature x]
forcecolor  = [0 0.4 1]; % color RGB

motor = [0.1, 0.05, .05, body(1)]; % motor
motorcolor = [0 0 .2];

h = [];

% make body
h(end+1) = rrect(body,bodycolor);

% make force
h(end+1) = rrect(force1(1:3),forcecolor);
twist(h(end), force1(4), body(2)/2 + force1(1), pi/2)
h(end+1) = rrect(force2(1:3),forcecolor);
twist(h(end), -force2(4), body(2)/2 + force2(1), pi/2)

% set edge color
ecolor  = get(h,'edgecolor');
ecolor  = cellfun(@(x) (x+4)/4,ecolor,'UniformOutput',false);
set(h, {'edgecolor'}, ecolor)

% make motor
h(end+1) = rrect(motor(1:3), motorcolor);
twist(h(end), body(1), body(2), 0)
h(end+1) = rrect(motor(1:3), motorcolor);
twist(h(end), -body(1), body(2), 0)

twist(h,x(1),x(2),x(3))

function twist(obj,x,y,theta)
% a planar twist: rotate object by theta, then translate by (x,y)
i = 1i;
if nargin == 3
   theta = 0;
end
for h = obj
   Z = get(h,'xdata') + i*get(h,'ydata');
   Z = Z * exp(i*theta);
   Z = Z + (x + i*y);
   set(h,'xdata',real(Z),'ydata',imag(Z));
end

function h = rrect(wlc, color)
% draw a rounded rectangle (using complex numbers and a kronecker sum :-)

N        = 25; % number of points per corner

width    = wlc(1);
length   = wlc(2);
curve    = wlc(3);

a        = linspace(0,2*pi,4*N);
circle   = curve*exp(1i*a);
width    = width-curve;
length   = length-curve;
rect1    = diag(width*[1 -1 -1 1] + 1i*length *[1 1 -1 -1]);
rectN    = sum(kron(rect1, ones(1,N)), 1) ;
rr       = circle + rectN;
rr       = [rr rr(1)]; % close the curve

h        = patch(real(rr),imag(rr),color);

% utility functions: singleton-expanded addition and multiplication
