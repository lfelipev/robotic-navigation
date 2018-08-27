clear;
clc;

map = imread('mapa.png');
[h, w, ~] = size(map);

robot_size = 2;
q_goal = [663, 310];

map = insertShape(map, 'FilledCircle', [q_goal(1) q_goal(2) robot_size], 'Color', 'green');

q_start = [27, 88];
zeta = 0.005;
d_star = 2;
Q_star = 50;
n = 1;
alpha = 1;
tolerance = 0.00001;
q = q_start;
grad_U = 1;

while norm(grad_U) > tolerance
    d = norm(q - q_goal);
    if(d <= d_star)
        grad_U_att = zeta * (q - q_goal);
    else
        grad_U_att = d_star * zeta * (q - q_goal) / d;
    end
    
    d = Q_star;
    
    range_y = [q(2) - (Q_star/2), q(2) + (Q_star/2)];
    range_x = [q(1) - (Q_star/2), q(1) + (Q_star/2)];
    
    
    if(range_y(1) <= 0)
        range_y(1) = 1;
    end
    if(range_x(1) <= 0)
        range_x(1) = 1;
    end
    if(range_y(2) > h)
        range_y(2) = h;
    end
    if(range_x(2) > w)
        range_x(2) = w;
    end
    
    for y0 = round(range_y(1))+1:round(range_y(2))
        for x0 = round(range_x(1)):round(range_x(2))
           
            if(map(y0, x0, :) == [0, 0, 0])
                d_i = norm(q - [x0, y0]);
                
                if(d_i < d)
                    d = d_i;
                    c = [x0, y0];
                end
            end
        end
    end
    
    if (d < Q_star)
        grad_d = (q - c)/d;
        grad_U_rep = n * ((1/Q_star) - (1/d)) * (1/(d^2)) * grad_d;
    else
        grad_U_rep = 0;
    end
     
    grad_U = grad_U_att + grad_U_rep;
    
    if(norm(q- q_goal) > 10)
        alpha = norm(q - q_goal) - (1/d);
    else
        alpha = d;
    end
    q = q + alpha * (-grad_U);
    
    map = insertShape(map, 'FilledCircle',[q(1) q(2) robot_size], 'Color', 'red');
    image(map)
    pause
end
        
                
    