close all
clear all
clc

% Read the map and set the height and width
map = imread('mapa.png');
[h, w, ~] = size(map);

% Define the goal location
q_goal = [65, 41];

% Define the parameters
% zeta to scale the effect of the attractive potential
% n as the gain of the repulsive gradient
% Q to allow the robot to ignore obstacles far away from it
zeta = 0.05;
d_star = 2;
Q_star = 50;
n = 5;

% Attractive potential (combined)
U_att = zeros(h, w);

for i = 1:h
    for j = 1:w
        % Quadratic function d²(q, q_goal)
        d = sqrt((q_goal(1)-i)^2 + (q_goal(2)-j)^2);
        
        if(d <= d_star)
            U_att(i, j) = 0.5 * zeta * (d^2);
        else
            U_att(i,j) = (d_star * zeta * d) - (0.5 * zeta * (d_star)^2);
        end
    end
end

[X, Y] = meshgrid(1:w, 1:h);
figure(1);
mesh(X, Y, U_att);

%% Repulsive potential
U_rep = zeros(h,w);

for j= 1:h
    for i = 1:w
        d = Q_star;
        
        range_x = [i - (Q_star/2), i + (Q_star/2)];
        range_y = [j - (Q_star/2), j + (Q_star/2)];
        
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
        
        % D is the distance to the closest obstacle D(q)
        for y0 = range_y(1):range_y(2)
            for x0 = range_x(1):range_x(2)
                if(map(y0, x0, 1) < 255)
                    d_i = sqrt((i-x0)^2 + (j-y0)^2);
                    
                    if(d_i < d)
                        d = d_i;
                    end
                end
            end
        end
        
        % The repulsive potential formula
        U_rep(j, i) = 0.5 * n * ((1/d) - (1/Q_star))^2;
        
        sat = 50;
        if(U_rep(j,i) > sat)
            U_rep(j, i) = sat;
        end
    end
end

figure(2);
mesh(X, Y, U_rep);

%%
% Sum the potential field
figure(3);
mesh(X, Y, U_att + U_rep);

        
    
    