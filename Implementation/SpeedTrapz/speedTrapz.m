close all
dt = 1/30;
current_velocity = 1;
current_position = 0;
target_speed  = 0.5;
target_position = 10;
cruise_speed = 2;
max_acc = 1;
max_speed = 2;

next_speed = 0;

figure 
h = animatedline; 

i = 0;
while abs(current_position) < abs(target_position)
    i = i+1;
    addpoints(h, i, current_velocity) 
    drawnow update 
    
    current_speed = abs(current_velocity);
    pos_error = target_position - current_position;    
    distance_to_reach_speed = abs(target_speed^2 - current_velocity^2)/(2 * max_acc);
        
    if abs(pos_error) - distance_to_reach_speed < 2 * max_acc * dt
        next_speed = current_speed - max_acc * dt;
        if abs(pos_error) < 2 * max_acc * dt                
            next_speed = target_speed;
        end
    else
        next_speed = current_speed + max_acc * dt;
        if next_speed >= cruise_speed
            next_speed = cruise_speed;
        end
    end
    
    current_velocity = next_speed * sign(pos_error);
    current_position = current_position + current_velocity * dt;
        
end