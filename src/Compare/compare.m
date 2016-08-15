function [td tt te] = compare_of()


%compare three different optimization functions
% rng(59);
% n_obs = 55; %number of static obstacles
% obs = rand(n_obs,2)*90+5; %obstacle locations
% rng(4); %for partially random obstacle size
% obs_rad = 3 +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%
%close all;
%import data
path_e = importdata('path_e.txt');
start_e = importdata('start_e.txt');
path_t = importdata('path_t.txt');
start_t = importdata('start_t.txt');
path_d = importdata('path_d.txt');
start_d = importdata('start_d.txt');


global n_obs obs obs_rad uav_finite_size uav_ws delta_t;


hold on

%tick labels
set(gca,'XTickLabel','')
set(gca,'YTickLabel','')

%plot landing area
cx = 50;
lr = 15;
xf = [100, 100];

cs = 2*lr/cx;
x = xf(1) - lr : cs : xf(1)+ lr;
y =  (lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %top part of circle
y1 = -(lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %bottom part of circle

plot(x,y,'g--');
plot(x,y1,'g--');

%square axes
axis square

for j = 1 : 3
    
    if j == 1
        Path_bez = path_e;
        path_start = start_e;
    elseif j == 2
        Path_bez = path_t;
        path_start = start_t;
    else
        Path_bez = path_d;
        path_start = start_d;
    end
    
    %plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, 0.5, 0]); %plots path of UAV
    
    if uav_finite_size == 1
        for i = 1 : length(path_start)
            x = path_start(i,1) - uav_ws : 0.05 : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle
            
            if j == 1
                
                plot(x,y,'r');
                plot(x,y1,'r');
            elseif j == 2
                
                plot(x,y,'g');
                plot(x,y1,'g');
            else
                
                plot(x,y,'b');
                plot(x,y1,'b');
            end
        end
    end
    
    if j == 1
        plot(Path_bez(:,1),Path_bez(:,2),'r'); %
        
    elseif j == 2
        plot(Path_bez(:,1),Path_bez(:,2),'g'); %g
        
    else
        plot(Path_bez(:,1),Path_bez(:,2),'b'); %y
        
    end
    
end

%plot static obstacles
for i = 1 : n_obs
    
   
    plot(obs(i,1),obs(i,2),'xk'); % staic obstacles' centers
    cs = 2*obs_rad(i)/cx;
    x = obs(i,1) - obs_rad(i) : cs : obs(i,1)+ obs_rad(i);
    y =  (obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %top part of circle
    y1 = -(obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %bottom part of circle
    
    plot(x,y,'k');
    plot(x,y1,'k');
end
xlim([0 100])
ylim([0 100])

title('Red = Energy, Green = Time, Blue = Distance');

hold off;

% Comparisons [ energy, time, distance ] 

%compare total distance traveled by UAV
td = zeros(3,1);
td(1) = evaluate_solution(path_e);
td(2) = evaluate_solution(path_t);
td(3) = evaluate_solution(path_d);

%compare time elapsed for each path
delta_time = 0.5*delta_t;

tt = zeros(3,1);
tt(1) = delta_time*length(path_e);
tt(2) = delta_time*length(path_t);
tt(3) = delta_time*length(path_d);

%compare average energy use
V = zeros(3,1);
eta = zeros(3,1);

for i = 1 : 3
   V(i) = td(i)/tt(i);
   
   eta(i) = calc_eff(V(i));
end

%parameter values
rho = 1.225; %air density
f = .2;   %equivalent parasite area
W = 10.0; %weight of aircraft
b = .2;   %span
e = 0.9; %Oswald's efficiency factor


%Defined in paper (2nd column, page 2)
A = rho*f/(2*W);
B = 2*W/(rho*b^2*pi*e);

%calculate l_d
for i = 1 : length(V)
    
    d_l(i) = A*V(i)^2 + B/V(i)^2; % we want to maximize l_d, or minimize d_l
    
    l_d(i) = d_l(i)^(-1);
    
end

%calculate energy use
te = zeros(3,1);

te(1) = d_l(1)*td(1)/eta(1);
te(2) = d_l(2)*td(2)/eta(2);
te(3) = d_l(3)*td(3)/eta(3);

end
