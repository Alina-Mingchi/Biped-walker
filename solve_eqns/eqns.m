function dy = eqns(t, y, y0, step_number, parameters)

% y0 is the states right after impact

global temp_step;
global var_noise;
global internal_noise_flag;
global external_force;
global sigma;
global timer;

%internal noise is constant for each gait cycle
if temp_step == step_number
	temp_step = temp_step + 1;
	var_noise = 3*sigma*randn();
end

%internal noise for q1
if internal_noise_flag == 1
    y(1) = y(1) + var_noise;
end

%internal noise for q2
if internal_noise_flag == 2
    y(2) = y(2) + var_noise;
end

%internal noise for q3
if internal_noise_flag == 3
    y(3) = y(3) + var_noise;
end

%internal noise for q4
if internal_noise_flag == 4
    y(4) = y(4) + var_noise;
end

%internal noise for q5
if internal_noise_flag == 5
    y(5) = y(5) + var_noise;
end

%internal noise for q6
if internal_noise_flag == 6
    y(6) = y(6) + var_noise;
end


% states of the robot
q = [y(1); y(2); y(3)];
dq = [y(4); y(5); y(6)];
theta = [y(7);y(8)];
x = [y(9);y(10)];

% states after impact
q0 = [y0(1); y0(2); y0(3)];
dq0 = [y0(4); y0(5); y0(6)];
theta0 = [y0(7);y0(8)];
x0 = [y0(9);y0(10)];

% ACPO controller parameters
p1 = [parameters(1); parameters(2); parameters(3)];
p2 = [parameters(5); parameters(6); parameters(7)];
c = [parameters(4); parameters(8)];
gamma = [parameters(9); parameters(10)];
K = [parameters(11); parameters(12)];
omega = [parameters(13); parameters(14)];

% polynom computation
g1 = p1(1)*theta(1)^3+p1(2)*theta(1)^2+p1(3)*theta(1)+c(1);   
g2 = p2(1)*theta(2)^3+p2(2)*theta(2)^2+p2(3)*theta(2)+c(2);   

%derivative of polynom with respect to phase
dg1=3*p1(1)*theta(1)^2+2*p1(2)*theta(1)+p1(3); 
dg2=3*p2(1)*theta(2)^2+2*p2(2)*theta(2)+p2(3);

M = eval_M(q);
C = eval_C(q, dq);
G = eval_G(q);
B = eval_B();

params=parameters(15:end);

u = control(t, q, dq, x, params);

dy = zeros(10, 1);
dy(1) = y(4);   %dq1
dy(2) = y(5);   %dq2
dy(3) = y(6);   %dq3
dy(4:6) = M \ (-C*dq - G + B*u);    %ddq1/ddq2/ddq3

% External force implementation
F = external_force;
acc_dq1 = F*cos(y(1))*6/7;
if step_number >= 10 && t<timer+0.2
    dy(4) = dy(4) + acc_dq1;
end

dy(7)=omega(1); 
dy(8)=omega(2);   

dy(9)=gamma(1)*(g1-x(1))+dg1*omega(1)+K(1);    %dx1
dy(10)=gamma(2)*(g2-x(2))+dg2*omega(2)+K(2);  %dx2

end