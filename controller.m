function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw


%Code given _des represents trajectory , in writeup all T is the
%qd{}.xxx_des

Kp_att = [24000 24000 700]';
Kd_att = [ 500 500 30]';

% Gains for Position control , Outer loop (X Y Z)
%Kp_pos = [16 70 200]';
%Kd_pos = [6  1 35]';
%16/6 - original
%58/10 

%Z 1800/100
% Y 60/9
% Kp_pos = [19 19 1500]';
% Kd_pos = [7  7  100]';

Kp_pos = [13 10 700]';
Kd_pos = [6 6 150]';

% Code eqn 11
Err_pos = (qd{qn}.pos_des - qd{qn}.pos);
Err_vel = (qd{qn}.vel_des - qd{qn}.vel);
rdd_des = qd{qn}.acc_des + Kp_pos.*Err_pos + Kd_pos.*Err_vel;

%Code eqn 14 

psi_des = qd{qn}.yaw_des;        %Determined independenty 
phi_des = (1/params.grav)   *   (rdd_des(1).*sin(qd{qn}.yaw_des) - rdd_des(2).*cos(qd{qn}.yaw_des));    
theta_des = (1/params.grav) *   (rdd_des(1).*cos(qd{qn}.yaw_des) + rdd_des(2).*sin(qd{qn}.yaw_des));   

%Code edn 15
p_des = 0 ;
q_des = 0 ;
r_des = qd{qn}.yawdot_des;

% Thurst
F    = params.mass*params.grav + params.mass*rdd_des(3);

% Moment
M    =  params.I*[Kp_att(1)*(phi_des - qd{qn}.euler(1)) + Kd_att(1)*(p_des - qd{qn}.omega(1));
                  Kp_att(2)*(theta_des - qd{qn}.euler(2)) + Kd_att(2)*(q_des - qd{qn}.omega(2));
                  Kp_att(3)*(psi_des - qd{qn}.euler(3)) + Kd_att(3)*(r_des - qd{qn}.omega(3));];
        
% %%
% %Test the attitude
% persistent phi_diff theta_diff psi_diff;
% 
% % Upon the first call we need to initialize the variables.
% if isempty(phi_diff)
%     phi_diff = 0;
%     theta_diff = 0;
%     psi_diff = 0 ;
% end
%               
% 
%               
% phi_diff = phi_diff + sqrt((phi_des - qd{qn}.euler(1))^2);
% theta_diff = theta_diff + sqrt((theta_des - qd{qn}.euler(2))^2)
% psi_diff = psi_diff +sqrt((psi_des - qd{qn}.euler(3))^2);
%  
% 
% 
% persistent gd;
% gd = [gd; t, theta_des, qd{qn}.euler(2)];  % for graphing
% 
% ..... and at some point you break to graph:
%   if t > 1
%      figure(2)
%      %Desired in red
%      plot(gd(:, 1), gd(:, 2), 'r')
%      hold on
%      %Actual in blue
%      plot(gd(:,1), gd(:, 3), 'b');
%      hold off
%      legend('Desired angle', 'Actual angle')
%   end
% 
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
