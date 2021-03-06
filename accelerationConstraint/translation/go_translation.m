%加速方式路程和时间
%控制方式默认SpeedUp使用时间约束，SpeedDown使用距离约束
%goForward function, describe : This is a function of chassis action
period = 0.02;
%约束条件变量
MIN_EXPECTED_SPEED = 0.05;
MAX_EXPECTED_SPEED = 0.3;
MIN_SHOWLY_DISTANCE = 0.3;
MAX_SHOWLY_DISTANCE = 0.5;
USE_EXP_CONSTRAINT = 1;
FILTER_TIME = 2.0;
%初始化变量
s = 0.0;
t = 0.0;
global_s = 0.0;
global_t = 0.0;
start_velocity = 0.0;    %局部控制开始时间速度
linear_velocity = 0.0;   %机器人当前时间速度
is_running = 1;
len = 4.0; %输入量，默认为位置控制
show_distance = len * 0.3;
speed_up_flag = 0;
speed_down_flag = 0;
show_distance = min(show_distance, MAX_SHOWLY_DISTANCE);
show_distance = max(show_distance, MIN_SHOWLY_DISTANCE);
close all;
hold on;
while is_running
    %update local timestamp and drive distance 
    global_t = global_t + period;
    global_s = global_s + linear_velocity * period;
    plot(global_t, global_s, 'c--.');
    %judge arrived target 
    if global_s >= (len - 0.02)
        is_running = 0;
        linear_velocity = 0; 
        continue;
        % should reset controller 
    end
    %smooth speed condition, is speed up or speed down 
    if (global_s >= (len - show_distance)) && (speed_down_flag == 0)
        speed_down_flag = 1;
        s = 0;
        t = 0;
        start_velocity = linear_velocity;
    elseif (linear_velocity < MAX_EXPECTED_SPEED) && (speed_up_flag == 0)
        speed_up_flag = 1;
        s = 0;
        t = 0;
        start_velocity = linear_velocity;
    end
    if speed_down_flag
        s = s + linear_velocity * period;
        t = t + period;  
        if USE_EXP_CONSTRAINT  % exp method
            rate = (1-exp(-(s)/show_distance)) / (1-exp(-1));
            rate = max(0.0, rate); % limit rate map to 0~1
            rate = min(1.0, rate);
            linear_velocity = start_velocity + (MIN_EXPECTED_SPEED - start_velocity) * rate;    
        else %scale method
            diff_speed = (start_velocity - MIN_EXPECTED_SPEED) / 2 + MIN_EXPECTED_SPEED;
            a = (start_velocity - MIN_EXPECTED_SPEED) / ((show_distance / diff_speed) * 50); % t 
            linear_velocity = linear_velocity - a;
            linear_velocity = max(linear_velocity, MIN_EXPECTED_SPEED);
        end
        %public velocity command 
        plot(global_t, linear_velocity, 'b--.', global_t, s, 'k--.');
    elseif speed_up_flag
        s = s + linear_velocity * period;
		t = t + period;
        if USE_EXP_CONSTRAINT
            %exp method
            rate = (1-exp(-(t)/FILTER_TIME)) / (1-exp(-1));
            rate = max(0.0, rate); % limit rate map to 0~1
            rate = min(1.0, rate);
            linear_velocity = start_velocity + (MAX_EXPECTED_SPEED - start_velocity) * rate;
        else
            %scale method 
            linear_velocity = linear_velocity + (MAX_EXPECTED_SPEED - start_velocity) / (FILTER_TIME * 50);
            linear_velocity = min(linear_velocity, MAX_EXPECTED_SPEED);
        end      
    %public velocity command 
        plot(global_t, linear_velocity, 'r--.', global_t, s, 'g--.');
    end
end

