close all;

path_t = importdata('path_t.txt');
start_t = importdata('start_t.txt');
path_t_opt = importdata('path_t_opt.txt');
start_t_opt = importdata('start_t_opt.txt');

path_e = importdata('path_e.txt');
start_e = importdata('start_e.txt');
path_e_opt = importdata('path_e_opt.txt');
start_e_opt = importdata('start_e_opt.txt');

path_d = importdata('path_d.txt');
start_d = importdata('start_d.txt');
path_d_opt = importdata('path_d_opt.txt');
start_d_opt = importdata('start_d_opt.txt');


global n_obs obs obs_rad uav_finite_size uav_ws delta_t t;

%t = 1, e = 2, d = 3
comp = 2;

hold on

for j = 1 : 2
    
    if j == 1
        if comp == 1
        Path_bez = path_t;
        path_start = start_t;
        elseif comp == 2
         Path_bez = path_e;
        path_start = start_e;   
        else
          Path_bez = path_d;
        path_start = start_d;  
        end
    elseif j == 2
        if comp == 1
        Path_bez = path_t_opt;
        path_start = start_t_opt;
        elseif comp == 2
         Path_bez = path_e_opt;
        path_start = start_e_opt;   
        else
          Path_bez = path_d_opt;
        path_start = start_d_opt;  
        end
    end
    
    %plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, 0.5, 0]); %plots path of UAV
    
    if uav_finite_size == 1
        for i = 1 : length(path_start)
            x = path_start(i,1) - uav_ws : 0.001 : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle
            
            if j == 1
                
                plot(x,y,'r');
                plot(x,y1,'r');
            elseif j == 2
                
                plot(x,y,'g');
                plot(x,y1,'g');
            else
                
                plot(x,y,'y');
                plot(x,y1,'y');
            end
        end
    end
    
    if j == 1
        plot(Path_bez(:,1),Path_bez(:,2),'r'); %
        
    elseif j == 2
        plot(Path_bez(:,1),Path_bez(:,2),'g'); %g

    end
    
end

%plot static obstacles
for j = 1 : n_obs
    
    
    plot(obs(j,1),obs(j,2),'xb'); % static obstacles' centers
    x = obs(j,1) - obs_rad(j) : 0.001 : obs(j,1)+ obs_rad(j);
    y =  (obs_rad(j)^2 - (x - obs(j,1)).^2).^0.5 + obs(j,2); %top part of circle
    y1 = -(obs_rad(j)^2 - (x - obs(j,1)).^2).^0.5 + obs(j,2); %bottom part of circle
    
    plot(x,y,'b');
    plot(x,y1,'b');
end
xlim([0 100])
ylim([0 100])

%title('Optimal path is green; planned path is red');
%title('Comparison to Optimal Time Path');
%title('Comparison to Optimal Energy Use Path');
%legend('Planned Path','Optimal Path');

hold off;