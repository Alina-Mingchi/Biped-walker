%clc;
%clear;
%close all;

%% run simulation
q0 = [pi/9; -pi/9; 0];
dq0 = [0; 0; 8];
x0 = [43.8512; -26.5132];
num_steps = 15;
global temp_step;
temp_step = 1;
default_parameters = control_hyper_parameters();
sln = solve_eqns(q0, dq0, x0, num_steps, default_parameters);
animate(sln);
analyse(sln, default_parameters, true);