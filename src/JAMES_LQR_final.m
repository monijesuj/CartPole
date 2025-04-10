M = 0.5;
m = 0.2;
b = 0.1;
I = 0.02;
g = 9.81;
l = 0.3;

A_22 = (-(I+m*l^2)*b)/(I*(M+m)+M*m*l^2);
A_23 = (m^2 * g * l^2)/ (I*(M+m) + M*m* l^2);
A_42 = (-m*l*b)/(I*(M+m) + M*m*l^2);
A_43 = ((m*g*l)*(M+m))/(I*(M+m) + M*m*l^2);

B_2 = (I + m*l^2)/(I*(M+m) + M*m*l^2);
B_4 = (m*l)/(I*(M+m) + M*m*l^2);

A = [0 1 0 0;
    0 A_22 A_23 0;
    0 0 0 1;
    0 A_42 A_43 0];
B = [0;
    B_2;
    0;
    B_4];
C = [1 0 0 0;
    0 0 1 0];
D = [0;
    0];
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

% GET STATE SPACE SYSTEM
sys_ss = ss(A,B,C,D);

% DISCRETIZATION OF STATE SPACE SYSTEM
Ts = 1/100;
sys_d = c2d(sys_ss,Ts,'zoh');

% TEST Controllability and Observability
co = ctrb(sys_d.A, sys_d.B);
ob = obsv(sys_d.A, sys_d.C);

controllability = rank(co)
observability = rank(ob)

%% LQR 
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;
Q = Cd'*Cd;
R = 1;

% DO LQR
K = dlqr(Ad, Bd, Q, R);
Ac = (Ad - Bd*K);
Bc = Bd;
Cc = Cd;
Dc = Dd;

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

% STATE SPACE OF THE SYSTEM
sys_cl = ss(Ac,Bc,Cc,Dc, Ts);

t = 0:0.01:20;
r = 0.2 * ones(size(0.5*t));
[y,t,x] = lsim(sys_cl, r, t, [0; 0; 0.5; 0]);



[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Digital LQR Control')

%% Animation 

figure;
ax_anim = axes('XLim',[-5, 5],'YLim',[-0.5, 2]);
hold on;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Animation of Cart-Pendulum System');

% Initialize graphics objects:
cart_plot = plot(NaN, NaN, 'k', 'LineWidth', 5);   % Cart as a horizontal line
pend_plot = plot(NaN, NaN, 'b-', 'LineWidth', 2);     % Pendulum as a line
ball_plot = plot(NaN, NaN, 'ro', 'MarkerSize', 8);    % Pendulum bob as a red circle

% Text annotations for time, angle (in degrees), and position:
time_text = text(2.75, 1.75, '', 'FontSize',10);
angle_text = text(2.75, 1.65, '', 'FontSize',10);
pos_text = text(2.75, 1.55, '', 'FontSize',10);

% Use the same time vector as in the simulation
nsim = length(t);
dt_anim = t(2) - t(1);  % Animation time step
speed_factor = 2; 
pause_duration = dt_anim * speed_factor; 

for i = 1:nsim
    % Extract simulation data: 
    cart_pos = y(i,1);
    pend_ang = y(i,2);
    
    % Calculate positions 
    cart_x = -cart_pos;  % Invert the cart position for the animation coordinate system
    % Cart is represented as a horizontal line segment centered at cart_x (length = 0.3 m)
    % The pendulum pivot is at (cart_x, 0)
    pendulum_x1 = cart_x + l * sin(pend_ang);
    pendulum_y1 = l * cos(pend_ang);
    
    % Update the cart plot (horizontal line segment)
    set(cart_plot, 'XData', [cart_x - 0.15, cart_x + 0.15], 'YData', [0, 0]);
    % Update the pendulum plot (line from the pivot to the bob)
    set(pend_plot, 'XData', [cart_x, pendulum_x1], 'YData', [0, pendulum_y1]);
    % Update the pendulum bob (marker)
    set(ball_plot, 'XData', pendulum_x1, 'YData', pendulum_y1);
    
    % Update text annotations
    set(time_text, 'String', sprintf('Time: %.2f', t(i)));
    set(angle_text, 'String', sprintf('Angle: %.2f', pend_ang*57.3)); % Convert rad to deg
    set(pos_text, 'String', sprintf('Pos: %.2f', -cart_pos));
    
    drawnow;
    pause(pause_duration);
end

%% LQR balanced at about 5s
%% Total control effort: 9.6217