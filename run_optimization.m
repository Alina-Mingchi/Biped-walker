%clc;
%clear;
close all;

%% optimize
% optimize the initial conditions and controller hyper parameters
q0 = [pi/9; -pi/9; 0];
dq0 = [0; 0; 8];
x0 = [43.8512; -26.5132];
y0 = [q0; dq0; x0; control_hyper_parameters()];

% use fminsearch and optimset to control the MaxIter
% stop either at 500 iterations or if 2 consecutive solutions change less
% than 1e-4
options = optimset('PlotFcns','optimplotfval','MaxIter',500,'TolX',1e-4);
x = fminsearch(@optimization_fun,y0,options);

%% simulate solution

% extract parameters
q0 = x(1:3);
dq0 = x(4:6);
x0 = x(7:8);
y_opt = x(9:end);

% simulate
num_steps = 30;
sln = solve_eqns(q0, dq0, x0, num_steps, y_opt);
animate(sln);
results = analyse(sln, y_opt, true);