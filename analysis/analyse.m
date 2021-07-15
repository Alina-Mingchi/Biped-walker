function results = analyse(sln, parameters, to_plot)

% calculate gait quality metrics (distance, step frequency, step length,
% velocity, etc.)
results=zeros(1,3);

% intialization of the vectors
q1=[];
q2=[];
q3=[];
dq1=[];
dq2=[];
dq3=[];
x_hip=[];
z_hip=[];
dx_hip=[];
dxmean_hip=[];
t=[];
u1=[];
u2=[];
frq=[];
dist=0;

% main loop across each gait cycle
for i=1:length(sln.Y)
    temp=sln.Y{i};
    time_gait=sln.T{i};
    temp2=[];
    [a,~]=size(temp);
    
    % time vector
    t=[t;sln.T{i}];
    
    % angle vectors
    q1=[q1;temp(:,1)];
    q2=[q2;temp(:,2)];
    q3=[q3;temp(:,3)];
    dq1=[dq1;temp(:,4)];
    dq2=[dq2;temp(:,5)];
    dq3=[dq3;temp(:,6)];
    
    % frequency vector
    frq=[frq;1/(time_gait(end)-time_gait(1))];
    
    u1int=[];
    u2int=[];
    
    % second loop across each index of a gait cycle
    for j=1:a
        
        [x_h, z_h, dx_h, ~] = kin_hip(temp(j,1:3), temp(j,4:6));
        
        % hip vectors
        x_hip=[x_hip;x_h];
        z_hip=[z_hip;z_h];
        dx_hip=[dx_hip;dx_h];
        temp2=[temp2;dx_h];
        
        % calculate actuation
        u = control(t, temp(j,1:3), temp(j,4:6), temp(j,7:8), parameters);
        u1=[u1;u(1)];
        u2=[u2;u(2)];
    end
    
    % only for first gait cycle
    if i==1
        
        q0=temp(1,1:3);
        dq0=temp(1,4:6);
        
        % substract the offset of the initial position of the swing foot
        [x0_swf, ~, ~, ~] = kin_swf(q0, dq0);
        [x_swf, ~, ~, ~] = kin_swf(temp(end,1:3), temp(end,4:6));
        lambda=x_swf-x0_swf; %step length
        
        % substract the offset of the initial position of the hip
        [d0, ~, ~, ~] = kin_hip(temp(1,1:3), temp(1,4:6));
        [df, ~, ~, ~] = kin_hip(temp(end,1:3), temp(end,4:6));
        dist = df-d0; % x_hip dist 
    else
        
        [x_swf, ~, ~, ~] = kin_swf(temp(end,1:3), temp(end,4:6));
        lambda=[lambda;x_swf];
        
        [d0, ~, ~, ~] = kin_hip(temp(1,1:3), temp(1,4:6));
        [df, ~, ~, ~] = kin_hip(temp(end,1:3), temp(end,4:6));
        dist=dist+df-d0;
    end
    
    dxmean_hip=[dxmean_hip;mean(temp2)];
end

frq=frq(2:end); % remove the step frequency of the first gait cycle

effort=(1/(2*length(t)*30))*sum(u1.^2+u2.^2);

CoT=abs(effort/dist);

% output vector
results=[mean(lambda),mean(dx_hip),max(dx_hip),min(dx_hip),dist,effort,CoT,mean(z_hip),mean(frq)];


if to_plot
    % plot the angles
    figure
    subplot(2,2,1)
    plot(t,q1)
    hold on
    plot(t,q2)
    plot(t,q3)
    title('Angle')
    xlabel('time [s]')
    ylabel('Angle [rad]')
    legend('q_{1}','q_{2}','q_{3}')
    
    % plot the hip position
    subplot(2,2,2)
    plot(t,x_hip)
    hold on
    plot(t,z_hip)
    title('Hip position')
    xlabel('time [s]')
    ylabel('Hip position [m]')
    legend('x_{hip}','z_{hip}')
    
    % plot instantaneous and average velocity
    subplot(2,2,3)
    plot(t,dx_hip)
    title('Instantaneous velocity')
    xlabel('time [s]')
    ylabel('x_{hip} velocity [m/s]')
    
    subplot(2,2,4)
    plot(1:1:length(sln.Y),dxmean_hip)
    title('Averaged velocity')
    xlabel('Step number')
    ylabel('x_{hip} averaged velocity [m/s]')
    
    % plot projections of the limit cycle
    figure
    subplot(2,2,1)
    plot(q1,dq1)
    title('q_{1} limit cycle')
    xlabel('q_{1} [rad]')
    ylabel('q_{1}dot [rad/s]')
    
    subplot(2,2,2)
    plot(q2,dq2)
    title('q_{2} limit cycle')
    xlabel('q_{2} [rad]')
    ylabel('q_{1}dot [rad/s]')
    
    subplot(2,2,3)
    plot(q3,dq3)
    title('q_{1} limit cycle')
    xlabel('q_{1} [rad]')
    ylabel('q_{1}dot [rad/s]')
    
    % plot actuation
    subplot(2,2,4)
    plot(t,u1)
    hold on
    plot(t,u2)
    title('Actuation')
    xlabel('time [s]')
    ylabel('actuation')
    legend('u1','u2')
    
    % plot step length vs step number
    figure
    subplot(2,2,1)
    plot(1:1:length(sln.Y),lambda)
    title('Displacement for each cycle')
    xlabel('Step number')
    ylabel('Step length [m]')
    
    % plot step frequency vs step number
    subplot(2,2,2)
    plot(2:1:length(sln.Y),frq)
    title('Step frequency')
    xlabel('Step number')
    ylabel('Step frequency [Hz]')
end

end