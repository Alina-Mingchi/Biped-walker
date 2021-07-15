function objective_value = optimization_fun(parameters)

global w1;
global w2;
global w3;
global w4;
global w5;
global target_mean_velocity;
global target_height;
global target_frequency;
global target_step_length;

% extract parameters q0, dq0 and x
q0 = parameters(1:3);
dq0 = parameters(4:6);
x0 = parameters(7:8);
y = parameters(9:end);

% run simulation
num_steps = 30; % the higher the better, but slow
sln = solve_eqns(q0, dq0, x0, num_steps, y);
results = analyse(sln, y, false);

% calculate metrics such as distance, mean velocity and cost of transport
step_length=results(1);
mean_velocity = results(2);
CoT = results(7);
height=results(8);
frequency = results(9);

% by adding the distance and height term in the objective value we handle
% the case when model walks backwards and when the model falls
objective_value = w1*abs(target_mean_velocity-mean_velocity) + w2*CoT + ...
    w3*abs(target_height-height) + w4*abs(target_frequency-frequency) + ...
    w5*abs(step_length-target_step_length); 

end

