function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% 
 
x = 0 ;
y = 0 ;
z = 0;
tf  = 12;

h = 1; %x distance
Y = [0 sqrt(2) 0 -sqrt(2)  0];
Z = [0 sqrt(2) 2*sqrt(2) sqrt(2) 0];

x = h*t/tf;
xdot = h/tf;

if(t <= tf/4)
    y = Y(1) + (t - 0)*(Y(2) - Y(1))/(tf/4);
    z = Z(1) + (t - 0)*(Z(2) - Z(1))/(tf/4);
    ydot = (Y(2) - Y(1))/(tf/4);
    zdot = (Z(2) - Z(1))/(tf/4);
    
end

if((t <= 2*tf/4) && (t>tf/4))
    y = Y(2) + (t - tf/4)*(Y(3) - Y(2))/(tf/4);
    z = Z(2) + (t - tf/4)*(Z(3) - Z(2))/(tf/4);
    ydot = (Y(3) - Y(2))/(tf/4);
    zdot = (Z(3) - Z(2))/(tf/4);
end

if((t <= 3*tf/4) && (t>2*tf/4))
    y = Y(3) + (t - 2*tf/4)*(Y(4) - Y(3))/(tf/4);
    z = Z(3) + (t - 2*tf/4)*(Z(4) - Z(3))/(tf/4);
    ydot = (Y(4) - Y(3))/(tf/4);
    zdot = (Z(4) - Z(3))/(tf/4);
end

if((t <= 4*tf/4) && (t>3*tf/4))
    y = Y(4) + (t - 3*tf/4)*(Y(1) - Y(4))/(tf/4);
    z = Z(4) + (t - 3*tf/4)*(Z(1) - Z(4))/(tf/4);
    ydot = (Y(1) - Y(4))/(tf/4);
    zdot = (Z(1) - Z(4))/(tf/4);

end

if (t>tf)
    y = Y(1);
    z = Z(1);
    x = h;
    xdot = 0 ; 
    ydot = 0 ;
    zdot = 0 ;
end


pos = [x;y;z];
vel = [xdot; ydot; zdot];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
