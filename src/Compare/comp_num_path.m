%compare three different optimization functions
% rng(59);
% n_obs = 55; %number of static obstacles
% obs = rand(n_obs,2)*90+5; %obstacle locations
% rng(4); %for partially random obstacle size
% obs_rad = 3 +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%
close all;

%import data
first = 1;
last = 6;

for i = first : last
    if i == 1
        path_1 = importdata('path_1.txt');
        start_1 = importdata('start_1.txt');
    elseif i == 2
        path_2 = importdata('path_2.txt');
        start_2 = importdata('start_2.txt');
    elseif i == 3
        path_3 = importdata('path_3.txt');
        start_3 = importdata('start_3.txt');
    elseif i == 4
        path_4 = importdata('path_4.txt');
        start_4 = importdata('start_4.txt');
    elseif i == 5
        path_5 = importdata('path_5.txt');
        start_5 = importdata('start_5.txt');
    elseif i == 6
        path_6 = importdata('path_6.txt');
        start_6 = importdata('start_6.txt');
    end
end

global n_obs obs obs_rad uav_finite_size uav_ws delta_t;


hold on

for j = first : last
    
    if j == 1
        Path_bez = path_1;
        path_start = start_1;
    elseif j == 2
        Path_bez = path_2;
        path_start = start_2;
    elseif j == 3
        Path_bez = path_3;
        path_start = start_3;
    elseif j == 4
        Path_bez = path_4;
        path_start = start_4;
    elseif j == 5
        Path_bez = path_5;
        path_start = start_5;
    elseif j == 6
        Path_bez = path_6;
        path_start = start_6;
    end
    
    
    %plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, 0.5, 0]); %plots path of UAV
    
    if uav_finite_size == 1
        for i = 1 : length(path_start)
            x = path_start(i,1) - uav_ws : 0.001 : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle
            
            if j == 1
                plot(x,y,'y');
                plot(x,y1,'y');
                
            elseif j == 2
                plot(x,y,'m');
                plot(x,y1,'m');
                
            elseif j == 3
                plot(x,y,'c');
                plot(x,y1,'c');
                
            elseif j == 4
                plot(x,y,'r');
                plot(x,y1,'r');
                
            elseif j == 5
                plot(x,y,'g');
                plot(x,y1,'g');
                
            elseif j == 6
                plot(x,y,'b');
                plot(x,y1,'b');
                
            end
        end
    end
    
    if j == 1
        plot(Path_bez(:,1),Path_bez(:,2),'y');
    elseif j == 2
        plot(Path_bez(:,1),Path_bez(:,2),'m');
    elseif j == 3
        plot(Path_bez(:,1),Path_bez(:,2),'c');
    elseif j == 4
        plot(Path_bez(:,1),Path_bez(:,2),'r');
    elseif j == 5
        plot(Path_bez(:,1),Path_bez(:,2),'g');
    else
        plot(Path_bez(:,1),Path_bez(:,2),'b');
        
    end
    
end

%plot static obstacles
for j = 1 : n_obs
    
    
    plot(obs(j,1),obs(j,2),'xk'); % static obstacles' centers
    x = obs(j,1) - obs_rad(j) : 0.001 : obs(j,1)+ obs_rad(j);
    y =  (obs_rad(j)^2 - (x - obs(j,1)).^2).^0.5 + obs(j,2); %top part of circle
    y1 = -(obs_rad(j)^2 - (x - obs(j,1)).^2).^0.5 + obs(j,2); %bottom part of circle
    
    plot(x,y,'k');
    plot(x,y1,'k');
end
xlim([0 100])
ylim([0 100])

title('Yellow = 1, Magenta = 2, Cyan = 3, Red = 4, Green = 5, Blue = 6');

hold off;

% Comparisons [ energy, time, distance ]

%compare total distance traveled by UAV

td = zeros(6,1);
for i = first : last
    if i == 1
        td(i) = evaluate_solution(path_1);
    elseif i == 2
        td(i) = evaluate_solution(path_2);
    elseif i == 3
        td(i) = evaluate_solution(path_3);
    elseif i == 4
        td(i) = evaluate_solution(path_4);
    elseif i == 5
        td(i) = evaluate_solution(path_5);
    elseif i == 6
        td(i) = evaluate_solution(path_6);
        
    end
end
