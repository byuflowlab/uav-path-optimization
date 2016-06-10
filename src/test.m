close all;
rng(60);
n_obs = 50; %number of static obstacles
obs = rand(n_obs,2)*90+5; %obstacle locations
rng(4); %for partially random obstacle size
obs_rad = 4-1.0 +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%

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

xlim([0 100])
ylim([0 100])

%different dynamic obstacle cases
% one big object sp = 100,100; vel = -5,5; r = 15.

%big and small object, both coming from final destination, smaller object
%is faster
% small object sp = 90,90; vel = -8,-8; r = 5;
%large object  sp = 100,100; vel = -5,5; r = 15.

%two objects, offset, coming from final destination
% sp = 100,90 and 70,80; vel = -8,-8; r = 10.

%things to do
%make cases function for dynamic obstacles
%make motor efficiency plot vs. velocity
%use plot and l_d plot for energy use
%implement in energy use

%think about changing total distance function (add penalty function for
%curvy plots)
%then, current minimize distance to minimize time
%