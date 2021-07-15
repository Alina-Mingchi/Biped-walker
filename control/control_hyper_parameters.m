% You can set any hyper parameters of the control function here; you may or
% may not want to use the step_number as the input of the function. 
function parameters = control_hyper_parameters()
%% Feed-forward controller parameters (ACPO)
% desired pattern polynom coefficients
p11 = -2834.58947156198/(16^3); %1
p12 = 2488.52554386207/(16^2);  %2
p13 = -627.874108829808/16; %3
c1 = 41.851233370501170;    %4

p21 = 2476.42723499253/(16^3);  %5
p22 = -2105.87598945210/(16^2); %6
p23 = 480.781655983828/16;  %7
c2 = -26.513158994207433;   %8

% ACPO tracking rate
gamma1=1;   %9
gamma2=1;   %10

% ACPO offset
K1=0;   %11
K2=0;   %12

% ACPO intrinsic frequency
omega1=16;  %13
omega2=16;  %14

%% Feedback controller parameters (PD)
% proportionnal and derivative gain
kd1 = 77.05;    %15                     
kp1 = 457.5;    %16

kd2 = 5;    %17
kp2 = 161;  %18

% desired lean angle
alpha = 10.4 * pi / 180;    %19

parameters = [p11, p12, p13, c1, p21, p22, p23, c2, gamma1, gamma2, K1, K2, omega1, omega2, kd1, kp1, ...
    kd2, kp2, alpha]';
end
