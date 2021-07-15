function u = control(t, q, dq, x, parameters)

global cpg_percent;
% extract parameters
if parameters
    kp1 = parameters(2);
    kp2 = parameters(4);
    kd1 = parameters(1);
    kd2 = parameters(3);
    alpha = parameters(5);
else
    parameters = control_hyper_parameters();
    kp1 = parameters(2);
    kp2 = parameters(4);
    kd1 = parameters(1);
    kd2 = parameters(3);
    alpha = parameters(5);
end

y1=q(3)-alpha;
y2=-q(2)-q(1);
dy1=dq(3);
dy2=-dq(2)-dq(1);

beta = cpg_percent;
% controller for torso
uff1=x(1); % feedforward control
ufb1=kp1*y1+kd1*dy1; % feedback control
u1 = beta*uff1+(1-beta)*ufb1;

% controller for legs
uff2=x(2);  % feedforward control
ufb2=kp2*y2+kd2*dy2; % feedback control
u2 = beta*uff2+(1-beta)*ufb2;

% saturate the output torque
u = [u1; u2];
u = max(min(u, 30), -30); 

end