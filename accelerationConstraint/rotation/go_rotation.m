%加速方式路程和时间
%控制方式默认SpeedUp使用时间约束，SpeedDown使用距离约束
%goForward function, describe : This is a function of chassis action
period = 0.02;
%约束条件变量
MIN_EXPECTED_SPEED = 0.1 / pi * 180;
MAX_EXPECTED_SPEED = 1.3 / pi * 180;
MIN_SHOWLY_DISTANCE = 20;
MAX_SHOWLY_DISTANCE = 60;
USE_EXP_CONSTRAINT = 0;
FILTER_TIME = 3;
%初始化变量
s = 0.0;
t = 0.0;
global_s = 0.0;
global_t = 0.0;
start_velocity = 0.0;    %局部控制开始时间速度
angular_velocity = 0.0;   %机器人当前时间速度
is_running = 1;
len = -180; %输入量，默认为位置控制
if len > 0
    dir = 1;
else 
    dir = -1;
end
show_distance = abs(len * 0.3);
speed_up_flag = 0;
speed_down_flag = 0;
show_distance = min(show_distance, MAX_SHOWLY_DISTANCE);
show_distance = max(show_distance, MIN_SHOWLY_DISTANCE);
close all;
hold on;
while is_running
    %update local timestamp and drive distance 
    global_t = global_t + period;
    global_s = global_s + angular_velocity * period;
    plot(global_t, global_s * dir, 'c--.');
    %judge arrived target 
    if abs(global_s) >= (abs(len) - 1)
        is_running = 0;
        angular_velocity = 0; 
        continue;
        % should reset controller 
    end
    %smooth speed condition, is speed up or speed down 
    if (abs(global_s) >= (abs(len) - abs(show_distance))) && (speed_down_flag == 0)
        speed_down_flag = 1;
        s = 0;
        t = 0;
        start_velocity = angular_velocity;
    elseif (angular_velocity < MAX_EXPECTED_SPEED) && (speed_up_flag == 0)
        speed_up_flag = 1;
        s = 0;
        t = 0;
        start_velocity = angular_velocity;
    end
    if speed_down_flag
        s = s + angular_velocity * period;
        t = t + period;  
        if USE_EXP_CONSTRAINT  % exp method
            rate = (1-exp(-(abs(s))/show_distance)) / (1-exp(-1));
            rate = max(0.0, rate); % limit rate map to 0~1
            rate = min(1.0, rate);
            angular_velocity = start_velocity + (MIN_EXPECTED_SPEED - start_velocity) * rate;    
        else %scale method
            diff_speed = (start_velocity - MIN_EXPECTED_SPEED) / 2 + MIN_EXPECTED_SPEED;
            a = (start_velocity - MIN_EXPECTED_SPEED) / ((show_distance / diff_speed) * 50); % t 
            angular_velocity = angular_velocity - a;
            angular_velocity = max(angular_velocity, MIN_EXPECTED_SPEED);
        end
        %public velocity command 
        plot(global_t, angular_velocity * dir, 'b--.', global_t, s * dir, 'k--.');
    elseif speed_up_flag
        s = s + angular_velocity * period;
		t = t + period;
        if USE_EXP_CONSTRAINT
            %exp method
            rate = (1-exp(-(t)/FILTER_TIME)) / (1-exp(-1));
            rate = max(0.0, rate); % limit rate map to 0~1
            rate = min(1.0, rate);
            angular_velocity = start_velocity + (MAX_EXPECTED_SPEED - start_velocity) * rate;
        else
            %scale method 
            angular_velocity = angular_velocity + (MAX_EXPECTED_SPEED - start_velocity) / (FILTER_TIME * 50);
            angular_velocity = min(angular_velocity, MAX_EXPECTED_SPEED);
        end      
        %public velocity command 
        plot(global_t, angular_velocity * dir, 'r--.', global_t, s * dir, 'g--.');
    end
end

