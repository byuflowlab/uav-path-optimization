function [M] = make_movie(Path_bez)

global n_obsd obs_d_sp obs_d_v obs_d_s obs_d_cp Dynamic_Obstacles;
global n_obs; %number of obstacles
global obs; %positions of obstacles
global obs_rad; %radius of obstacles

global uav_ws;

global t;

M(length(Path_bez)) = struct('cdata',[],'colormap',[]);

hold on

%plot static obstacles
for j = 1 : n_obs
    
    plot(obs(j,1),obs(j,2),'xb'); % static obstacles' centers
    x = obs(j,1) - obs_rad(j) : 0.001 : obs(j,1)+ obs_rad(j);
    y =  (obs_rad(j)^2 - (x - obs(j,1)).^2).^0.5 + obs(j,2); %top part of circle
    y1 = -(obs_rad(j)^2 - (x - obs(j,1)).^2).^0.5 + obs(j,2); %bottom part of circle
    
    plot(x,y,'b');
    plot(x,y1,'b');
    
end

for i = 1 : length(Path_bez)
  
    %plot uav position and size
    
    x = Path_bez(i,1) - uav_ws : 0.001 : Path_bez(i,1)+ uav_ws;
    y =  (uav_ws^2 - (x - Path_bez(i,1)).^2).^0.5 + Path_bez(i,2); %top part of circle
    y1 = -(uav_ws^2 - (x - Path_bez(i,1)).^2).^0.5 + Path_bez(i,2); %bottom part of circle
    
    plot(x,y,'Color',[0, 0.5, 0]);
    plot(x,y1,'Color',[0, 0.5, 0]);
    
    xlim([0,100]);
    ylim([0,100]);
    
    %plot dynamic obstacles
    
    if Dynamic_Obstacles == 1
        
        %plot small square at center of dynamic obstacles at each time step
        for k = 1 : n_obsd
            
            plot(obs_d_v(k,1)*t(2)*(i-1) + obs_d_sp(k,1),obs_d_v(k,2)*t(2)*(i-1) + obs_d_sp(k,2),'rs');
            
        end
        %plot  dynamic obstacles as circles
        for k = 1 : n_obsd
            
            
            odh = [obs_d_v(k,1)*t(2)*(i-1)+obs_d_sp(k,1), obs_d_v(k,2)*t(2)*(i-1)+obs_d_sp(k,2) ]; % to make it easier to type
            
            x = odh(:,1) - obs_d_s(k) : 0.001 : odh(:,1)+ obs_d_s(k);
            y =  (obs_d_s(k)^2 - (x - odh(:,1)).^2).^0.5 + odh(:,2); %top part of circle
            y1 = -(obs_d_s(k)^2 - (x - odh(:,1)).^2).^0.5 + odh(:,2); %bottom part of circle
            
            plot(x,y,'r');
            plot(x,y1,'r');
        end
    end
    
    M(i) = getframe(gcf);
    
end

hold off

end