close all;
hold on
[path_x path_y] = textread('astar_path.txt');
[x y] = textread('simulate_path.txt');
%path_x = x;
%path_y = y;
%[path_x path_y] = textread('astar_path40cm.txt');
%path_x = path_x(200:400);
%path_y = path_y(200:400);
plot(path_x, path_y, 'bo')
sample_index = 1;
first_sample = 1;
bezier_index = 0;
bezier_x = [];
bezier_y = [];
while sample_index < (length(path_x) - 5)
    %obtain sample points
    if first_sample
        p0_x = path_x(sample_index);
        p0_y = path_y(sample_index);
        p1_x = path_x(sample_index+1);
        p1_y = path_y(sample_index+1);
        p2_x = path_x(sample_index+2);
        p2_y = path_y(sample_index+2);
        p3_x = path_x(sample_index+3);
        p3_y = path_y(sample_index+3);
    else
        p0_x = bezier_x(length(bezier_x));
        p0_y = bezier_y(length(bezier_y));
        p1_x = path_x(sample_index);
        p1_y = path_y(sample_index);
        p2_x = path_x(sample_index+1);
        p2_y = path_y(sample_index+1);
        p3_x = path_x(sample_index+2);
        p3_y = path_y(sample_index+2);
    end 
%     plot(p0_x, p0_y, 'bo')
%     plot(p1_x, p1_y, 'bo')
%     plot(p2_x, p2_y, 'bo')
%     plot(p3_x, p3_y, 'bo')
    %calculate bezier curve
    node = 0:0.2:0.8;
    for t = 1:length(node) 
        bezier_x(t+bezier_index*length(node)) = p0_x*((1-node(t))^3) + 3*p1_x*node(t)*((1-node(t))^2) + 3*p2_x*(node(t)^2)*(1-node(t)) +p3_x*(node(t)^3);
        bezier_y(t+bezier_index*length(node)) = p0_y*((1-node(t))^3) + 3*p1_y*node(t)*((1-node(t))^2) + 3*p2_y*(node(t)^2)*(1-node(t)) + p3_y*(node(t)^3);
    end
%     plot(bezier_x, bezier_y, 'r.')
    if first_sample 
        sample_index = sample_index + 3;
    else
        sample_index = sample_index + 2;
    end
    bezier_index = bezier_index + 1;
    first_sample = 0;
end
plot(bezier_x, bezier_y, 'r.')

% obtain track, now we have get a track controller
% first we have face to next way point 
% and linear velocity = 0.0 angular velocity = 0.0
linear_velocity = 0.0;
angular_velocity = 0.0;
max_linear_velocity = 0.25; %0.2m/s
max_angular_velocity = 0.7; %~40度/s
current_index = 1;
next_index = 2;
pose_x = bezier_x(current_index);
pose_y = bezier_y(current_index);
phi = atan2(bezier_y(next_index) - bezier_y(current_index), bezier_x(next_index) - bezier_x(current_index)); 
control_frequency = 50;
dt = 1 / control_frequency;
control_step = 1;
initializing = 1;
controlling = 2;
control_x = [];
control_y = [];
pose_update = 1;
angular_accelerated_scale = 0.05; %0.02~0.03
angular_velocity_deq = [];
angular_velocity_index = 1;
linear_velocity_deq = [];
linear_velocity_index = 1;
while next_index < length(bezier_x)
    % simulate1 pose update;
    if abs(angular_velocity) < 0.00001
        pose_x = pose_x + linear_velocity * cos(phi) * dt;
        pose_y = pose_y + linear_velocity * sin(phi) * dt;
    else 
        radius = linear_velocity / angular_velocity;
        dphi = angular_velocity * dt;
        pose_x = pose_x - radius * sin(phi) + radius * sin(phi + dphi);
        pose_y = pose_y + radius * cos(phi) - radius * cos(phi + dphi);
    end
    control_x(pose_update) = pose_x;
    control_y(pose_update) = pose_y;
    pose_update = pose_update + 1;
    %figure(1)
    %hold on 
    %plot(pose_x, pose_y, 'g.')
    %plot(bezier_x(next_index), bezier_y(next_index), 'y.')
    % simulate2 velocity control
    if control_step == initializing
        dx = bezier_x(next_index) - pose_x; % obtain distance to next waypoint
        dy = bezier_y(next_index) - pose_y;
        distance_to_next = sqrt(dx^2 + dy^2);
        control_step = controlling;
    else % control_step == controlling
        % if finish
        dx = bezier_x(next_index) - pose_x; % obtain distance to next waypoint
        dy = bezier_y(next_index) - pose_y;
        distance = sqrt(dx^2 + dy^2);
        if (distance < (distance_to_next * 0.3)) || (distance < 0.15)
            next_index = next_index + 1;
            control_step = initializing;
        end 
        % else if fail 
        % else
        dx = bezier_x(next_index) - pose_x; 
        dy = bezier_y(next_index) - pose_y;
        expect = atan2(dy, dx);
        error = expect - phi; 
        expect_angular_velocity = error * 2.0;
        if expect_angular_velocity > max_angular_velocity
            expect_angular_velocity = max_angular_velocity;
        elseif expect_angular_velocity < -max_angular_velocity
            expect_angular_velocity = -max_angular_velocity;
        end
        
        if expect_angular_velocity > angular_velocity
            angular_velocity = angular_velocity + angular_accelerated_scale;
        elseif expect_angular_velocity < angular_velocity
            angular_velocity = angular_velocity - angular_accelerated_scale;
        end
        
        linear_velocity = max_linear_velocity * ((max_angular_velocity - abs(angular_velocity) * 0.7) / max_angular_velocity); %1-0.6=0.4 0.4*0.25=0.1m/s min
        angular_velocity_deq(angular_velocity_index) = angular_velocity;
        angular_velocity_index = angular_velocity_index + 1;
        linear_velocity_deq(linear_velocity_index) = linear_velocity;
        linear_velocity_index = linear_velocity_index + 1;
    end 
    % simulate3 state update
    phi = phi + angular_velocity * dt;
end
plot(control_x, control_y, 'g.')

figure(2)
hold on 
plot(linear_velocity_deq)
plot(angular_velocity_deq)
% 一次贝塞尔曲线。
% p0 = [0, 0]
% p1 = [2, 3]
% t = 0:0.01:1
% bx = []
% by = []
% for i = 1:length(t)
%     bx(i) = (1-i)*p0(1) + i*p1(1);
%     by(i) = (1-i)*p0(2) + i*p1(2);
% end
% plot(bx,by)

% 二次贝塞尔曲线
% p0 = [0, 0]
% p1 = [1, 1]
% p2 = [2, 6]
% node = 0:0.01:1;
% bx = []
% by = []
% for t = 1:length(node) 
%     bx(t) = (1-node(t))^2*p0(1)  + 2*node(t)*(1-node(t))*p1(1)+(node(t)^2)*p2(1);
%     by(t) = (1-node(t))^2*p0(2)  + 2*node(t)*(1-node(t))*p1(2)+(node(t)^2)*p2(2);
% end
% hold on 
% plot(p0(1),p0(2), 'ro')
% plot(p1(1),p1(2), 'go')
% plot(p2(1),p2(2), 'bo')
% plot(bx,by)

% 三次贝塞尔曲线
% p0 = [0, 0]
% p1 = [1, 1]
% p2 = [2, 6]
% p3 = [7, 4]
% node = 0:0.05:1;
% bx = []
% by = []
% for t = 1:length(node) 
%     bx(t) = p0(1)*((1-node(t))^3) + 3*p1(1)*node(t)*((1-node(t))^2) + 3*p2(1)*(node(t)^2)*(1-node(t)) + p3(1)*(node(t)^3);
%     by(t) = p0(2)*((1-node(t))^3) + 3*p1(2)*node(t)*((1-node(t))^2) + 3*p2(2)*(node(t)^2)*(1-node(t)) + p3(2)*(node(t)^3);
% end
% hold on 
% plot(p0(1),p0(2), 'ro')
% plot(p1(1),p1(2), 'go')
% plot(p2(1),p2(2), 'bo')
% plot(p3(1),p3(2), 'bo')
% plot(bx,by, 'r.')
