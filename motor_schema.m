clear
close all
clc
%% create circle
center = [0, 0]; % center of circle
radius = 5; % radius of circle
num_points = 8;
% angle
angles = linspace(pi/20, 2*pi, num_points+1);
angles = angles(1:end-1);
% coordinate of robot
x_coordinates = center(1) + radius * cos(angles);
y_coordinates = center(2) + radius * sin(angles);
%% start locations
xs = x_coordinates(1:num_points/2) ;
ys = y_coordinates(1:num_points/2);
%% goal locations
xg = x_coordinates(num_points/2+1:num_points);
yg = y_coordinates(num_points/2+1:num_points);
%% init
v_ref = 1;
C = [0 0];
%% param
S = 3; %sensing range
R = 0.2; %radius of robot
wg = 0.8; %weight of detect goal
wo = 0.8; %weight of detect collision
wr = 0.01; %weight of random walk
position_accuracy = 0.1;
dstar = 1;
%% Parameters related to kinematic model
error_theta_max = deg2rad(45);
v_max = 1;
Kp_omega = 1.5;
omega_max = 0.5*pi; 
%% process
figure(1); 
t = 1;
dT = 0.2;
t_max = 150;
n = num_points/2; %number of robot
% create matrix
X = cell(n, 1);
Y = cell(n, 1);
theta = zeros(1,n);
for i = 1:n
    X{i} = zeros(1,t_max);
    Y{i} = zeros(1,t_max);
end
for i = 1:n
    X{i}(1) = xs(i);
    Y{i}(1) = ys(i);
end
while t < t_max
    for i = 1:n 
        if (norm([xg(i) yg(i)] - [xs(i) ys(i)]) > position_accuracy )
            % Calculate detect goal
            if norm([xg(i) yg(i)]-[xs(i) ys(i)]) <= dstar
                v_goal =  ([xg(i) yg(i)]-[xs(i) ys(i)]);
            else 
                v_goal = ([xg(i) yg(i)]-[xs(i) ys(i)]/norm([xg(i) yg(i)]-[xs(i) ys(i)]));
            end
            % Calculate detect collision
             v_obs = 0;
            for j = 1:n
                dist = sqrt((xs(i) - xs(j))^2 + (ys(i) - ys(j))^2);
                if dist >= S || dist == 0
                    C = [0 0];
                else
                    C = (S-dist)/(S-R)*([xs(i) ys(i)]-[xs(j) ys(j)])/sqrt((xs(i) - xs(j))^2 + (ys(i) - ys(j))^2);
                end
                v_obs = (v_obs+C);
            end
            % Calculate random walk
            v_rand = [rand() rand()];
            % Calculate final velocity
            vel = wg * v_goal + wo * v_obs + wr * v_rand;
            xs(i) = xs(i) + vel(1) * dT;
            ys(i) = ys(i) + vel(2) * dT;
            
            X{i}(t) = xs(i);
            Y{i}(t) = ys(i);
            daspect([1 1 1]); 
            title('Motor schema swarm robot');
            xlabel('m') 
            ylabel('m') 
            xlim([-6,6]);  ylim([-6 6]);
            box on; hold on;
            plot(x_coordinates(i),y_coordinates(i),'*');
            plot(xg(i), yg(i), 'ob');
            plot([x_coordinates(i),X{i}(1:t)],[y_coordinates(i), Y{i}(1:t)],'Linewidth',2); % Plot traveled path
            drawnow;
            pause(dT);
        end
    end
    % Archive and plot it
    t = t + 1;
           
end   
    
t = t*dT;
disp("Travel time: " + t);


