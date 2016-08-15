function [td, tt, te] = compare_of(Path_Bez, Bez_points,optimize_energy_use,optimize_time)

%compare three different optimization functions
% rng(59);
% n_obs = 55; %number of +5static obstacles
% obs = rand(n_obs,2)*90; %obstacle locations
% rng(4); %for partially random obstacle size
% obs_rad = 3 +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%
%close all;
%import data
path_t = importdata('path_t.txt');
start_t = importdata('start_t.txt');
path_e = importdata('path_e.txt');
start_e = importdata('start_e.txt');
path_d = importdata('path_d.txt');
start_d = importdata('start_d.txt');


global n_obs obs obs_rad uav_finite_size uav_ws delta_t t;
global max_speed min_speed xf D_eta_opt;

make_plot = 0;

if make_plot == 1
hold on

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
        
    else
        plot(Path_bez(:,1),Path_bez(:,2),'y'); %y
        
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

title('Red = Energy, Green = Time, Yellow = Distance');

hold off;

end

% Comparisons [ energy, time, distance ] 

%compare total distance traveled by UAV
td = 0;

l_l = zeros(length(Bez_points)/2,1);
x_sp = [0,0];
xi = Bez_points;

for i = 1 : length(Bez_points)/2

    
    if i == 1
        p_prev = x_sp(1,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*x_sp(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            l_l(i) = l_l(i) + d;
            
            %change initial position
            p_prev = p;
        end
        
    else
        p_prev = xi(2*i-2,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            l_l(i) = l_l(i) + d;
            
            %change initial position
            p_prev = p;
        end
    end
    
end

total_length = 0;
for i = 2 : length(Path_Bez)
    total_length = total_length + norm(Path_Bez(i,:)-Path_Bez(i-1,:));
end
td = total_length + norm(Bez_points(length(Bez_points),:) - xf);


%compare time elapsed for each path
%delta_time = delta_t;

tt = (length(Bez_points))/2 + norm(Bez_points(length(Bez_points),:) - xf)/max_speed;

%compare energy use
V = zeros(length(l_l),1);
eta = zeros(length(l_l),1);

for i = 1 : length(l_l)
   V(i) = l_l(i)/1;
   
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
te = 0;

for i = 1 : length(l_l)
   te = te + d_l(i)*l_l(i)/eta(i); 
end

te = te + norm(Bez_points(length(Bez_points),:) - xf)*D_eta_opt;

end
