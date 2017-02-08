function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

tf = 13.6;
r = 5;
b = 2.5;
pos = [r*cos(2*pi*t/tf)         ;   r*sin(2*pi*t/tf)        ;   b*t/tf];
vel = [-r*2*pi/tf*sin(2*pi*t/tf);   r*2*pi/tf*cos(2*pi*t/tf);   b/tf];
acc = [-r*(2*pi/tf)^2*cos(2*pi*t/tf);-r*(2*pi/tf)^2*sin(2*pi*t/tf);0];
yaw = 0;
yawdot = 0;

if (t >= tf)
    pos = [r; 0; b];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end 

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
