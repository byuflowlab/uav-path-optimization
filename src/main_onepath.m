% ------- Main File ------ %
% Author : Bryce Ingersoll
% Institution: Brigham Young University, FLOW Lab
% Last Revised : 6/29/16
% ------------------------ %

clear; clc; close all;

%Add paths
addpath(genpath('.\Objective_Functions\'));
addpath(genpath('.\Constraints\'));
addpath(genpath('.\ColorPath\'));
addpath(genpath('.\Compare\'));
addpath(genpath('.\OptimalPathGuesses\'));
addpath(genpath('.\CalculateEnergyUse\'));

%profiling tools
profile on

%-------global variables----------%
global xf; %final position
global x0; %current starting pointPath_bez
global step_max; %max step distance
global step_min; %minimum step distance
global t; %parameterization variable
global n_obs; %number of obstacles
global obs; %positions of obstacles
global obs_rad; %radius of obstacles
global turn_r; %minimum turn radius
global Pmid; %needed to match derivatives
global num_path; %number of segments optimized
global x_new;
global Dynamic_Obstacles;
global x_next; %used in multi_start function
global uav_ws; %UAV wing span
global start;
global initial; % to calculate d_l_min
initial = 1;
global uav_finite_size;
global rho f W span eo;
global summer cool copper parula_c;
global obj_grad cons_grad ag acg;
global max_speed min_speed;
global lr D_eta_opt;

%------------Algorithm Options------------%
Dynamic_Obstacles = 0;

num_path = 3;              %Receding Horizon Approach (any number really, but 3 is standard)
ms_i = 1;                  %number of guesses for multi start (up to 8 for now, up to 3 for smart)
uav_finite_size = 1;       %input whether want to include UAV size
check_viability = 1;       %Exits if unable to find viable path

%Objective Function
optimize_energy_use = 0;    %changes which objective function is used
optimize_time =  1;         %if both are zero, then path length is optimized

max_func_evals = 100000;
max_iter = 50000;

% Plot Options
totl = 1;             %turn off tick labels
square_axes = 1;      %Square Axes
radar = 0;            %Plots UAV's limit of sight
linewidth = 3;        %Line width of traversed path segment
show_sp = 0;          %Plots P2 of Bezier curve
Show_Steps = 0;       %Needs to be turned on when Dynamic_Obstacles is turned on
show_end = 0;         %for calc_fig
compare_num_path = 0;
save_path = 1;        %save path data to use in compare
sds = 0;              %Allows a closer view of dynamic obstacle avoidance
cx = 50;

create_video = 1;          %saves the solutions of the multistart approach at each iteration

% Gradient Calculation Options
% obj_grad = 1;           %if this is 1 and below line is 0, complex step method will be used to calculate gradients
% analytic_gradients = 1;
% ag = analytic_gradients;
% 
% cons_grad = 1;          %if this is 1 and below line is 0, complex step method will be used to calculate gradients
% analytic_constraint_gradients = 1;
% acg = analytic_constraint_gradients;

%plot color options
speed_color = 1;         %use if you want color to represent speed
d_speed_color = 0;       %use if you want color to be discretized over path length
cb = 1;                  %color brightness
summer = 0;             % http://www.mathworks.com/help/matlab/ref/colormap.html#buq1hym
cool = 0;
copper = 0;
parula_c = 1;
color_bar = 1;
%----------------------------------------%


%-------------- one_path -----------------%

% -- obstacle fields used for opt_compare (rng 11-20) 40/4/3 -- %
%plan entire path
% to run this, first need to run using 3-4 num_path, save that path, and
% use that as your initial guess; also need to change number of num_path to
% match what was previously solved for
one_path = 1; %if this is on, need to set ms_i = 1
%planned vs. optimal paths
%rng(4); %49/4/3

rng(11); %40/4/3 ; d = num_path = 13, t = num_path = 10, e = num_path = 11
%rng(12); %40/4/3 ; d = num_path = 14, t = num_path = 10, e = num_path = 11
%rng(13); %40/4/3 ; d = num_path = 14, t = num_path = 11, e = num_path = 12
%rng(14); %40/4/3 ; d = num_path = 14, t = num_path = 11, e = num_path = 12
%rng(15); %40/4/3 ; d = num_path = 14, t = num_path = 11, e = num_path = 13
%rng(16); %40/4/3 ; d = num_path = 13, t = num_path = 10, e = num_path = 12
%rng(17); %40/4/3 ; d = num_path = 13, t = num_path = 10, e = num_path = 11
%rng(18); %40/4/3 ; d = num_path = 13, t = num_path = 10, e = num_path = 12
%rng(19); %40/4/3 ; d = num_path = 13, t = num_path = 10, e = num_path = 12
%rng(20); %40/4/3 ; d = num_path = 14, t = num_path = 11, e = num_path = 12

if one_path == 1
    num_path = 9;
    ms_i = 1;
    get_bez_points = @r12_t;
end
% ------------------------------------------------------------- %

l = 0;

%parameterization vector t
global delta_t;
t = linspace(0,1,10);
delta_t = t(2) - t(1);

%for plot_both function
%global Path_bez;

%----------------plane geometry/info----------------%
%UAV parameter values
rho = 1.225; %air density
f = .2;   %equivalent parasite area
W = 10; %weight of aircraft
span = .20;   %span
eo = 0.9; %Oswald's efficiency factor

turn_r = 5; %turn radius, m

%maximum/stall speed, m/s
max_speed = 15;
min_speed = 10;
if optimize_energy_use == 1
    min_speed = 10;
end

%transalte UAV information to fit with algorithm
step_max = max_speed; %/2;
step_min = min_speed; %/2;

%Wing span of UAV
if uav_finite_size == 1
    uav_ws = 1.0; %UAV wing span
else
    uav_ws = 0.001;
end

%starting/ending position of plane
x0 = [0,0];
xf = [100,100];
Bez_points = [];
lr = 15;
%--------------------------------------------------%

if optimize_energy_use == 1
    %Defined in paper (2nd column, page 2)
    A = rho*f/(2*W);
    B = 2*W/(rho*span^2*pi*eo);
    
    %find minimum d_l, and minimum efficiency
    if initial == 1
        V_possible = 0.1 : 0.01 : 25;
        
        for i = 1 : length(V_possible)
            
            D_L = A*V_possible(i)^2 + B/V_possible(i)^2; % we want to maximize l_d, or minimize d_l
            
            eta_pos = calc_eff(V_possible(i));
            
            %calculate D_L/eta
            D_eta(i) = D_L/eta_pos;
        end
        
        %find optimal D_eta
        D_eta_opt = min(D_eta);
        
    end
end
% --------------------------------- %


%-------static obstacle information---------%

n_obs = 40; %number of static obstacles
obs = rand(n_obs,2)*90+5; %obstacle locations
rng(4); %for partially random obstacle size
obs_rad = (4-uav_ws) +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%

%------dynamic obstacle information---------%
if Dynamic_Obstacles == 1
    
    global n_obsd obs_d_sp obs_d_v obs_d_s obs_d_cp;
    
    %choose 1-4 for cases (see function for description)
    [n_obsd, obs_d_sp, obs_d_s, obs_d_v]  = dyn_case(5);
    
    obs_d_s = obs_d_s-ones(n_obsd,1)*uav_ws; %size of obstacles, also used (5)
    obs_d_cp = obs_d_sp; %current position of obstacles
    obs_d_cp_hist(1,:,1) = obs_d_sp(1,:);
end
%-------------------------------------------%

% for make_video
if create_video == 1
    
    solution1 = [];
    solution2 = [];
    solution3 = [];
    solution4 = [];
    solution5 = [];
    
end

tic
%----------------- optimizer ---------- fmincon -----------------------%
%unused parts in fmincon
A = [];
b = [];
Aeq = [];
beq = [];
%lb = -10*ones(2*num_path,2);
%ub = 110*ones(2*num_path,2);
lb = [];
ub = [];

%Pmed initialization, used to match up derivatives between paths
Pmid = [-min_speed/2,-min_speed/2];
%Pmid = [-3,-3];

Path_bez = [];

path_start = [];

%initial guess(es)
start = 0;

xi = multi_start(ms_i);

%start
start = 1;

%x_new is not close to final position
x_new = zeros(2*num_path,2);


%-------------------------final optimization------------------%
%final guess
if one_path == 1
    x_guess_final = get_bez_points();
else
    x_guess_final = multi_start(ms_i);
end

%one path optimization for energy use
%if one_path == 1 && optimize_energy_use == 1
if optimize_energy_use == 1
    final_of = @onepath_e;
elseif optimize_time == 1
    final_of = @onepath_t;
else
    final_of = @onepath_d;
end


for i = 1 : ms_i %multistart approach to find best solution
    
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',10000,'MaxIter',100);
    %options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',500000,'MaxIter',100000);
    [x_final(:,:,i), fval] = fmincon(final_of, x_guess_final(:,:,i) , A, b, Aeq, beq, lb, ub, @onepath_cons,options);
    
end

% for compare_of function
if one_path == 1
    Bez_points = x_final;
end


x_fin = x_final;



%--------------------Final Plot-------------------------------%

%------------add last segment of path to total-----------%
for j = 1 : num_path
    
    if j == 1
        for i = 1 : length(t)
            
            path_mid(i,:) = (1-t(i))^2*x0(1,:) + 2*(1-t(i))*t(i)*x_fin(1,:)+t(i)^2*x_fin(2,:);
            
        end
    else
        for i = 1 : length(t)
            
            path_mid(i,:) = (1-t(i))^2*x_fin(2*j-2,:) + 2*(1-t(i))*t(i)*x_fin(2*j-1,:)+t(i)^2*x_fin(2*j,:);
        end
    end
    
    path_start = [path_start; path_mid(1,:)];
    Path_bez = [Path_bez; path_mid];
end


figure(l+1);
hold on

if square_axes == 1
    axis square
end

if totl == 1
    
    set(gca,'XTickLabel','')
    set(gca,'YTickLabel','')
    
end

%----------------plot UAV-------------------%
if color_bar == 1
    colorbar('southoutside','Ticks',[0,0.20,0.4,0.6,0.8,1],'TickLabels',{'V_{min}, 10 m/s','11 m/s','12 m/s','13 m/s','14 m/s','V_{max},15 m/s'},'fontsize',14);
end
if speed_color == 1
    
    num_segments = length(Path_bez)/length(t);
    num_bits = length(Path_bez)-1;
    
    segment_length = zeros(num_segments,1);
    bit_length = zeros(num_bits,1);
    
    segment = zeros(length(t),2,num_segments);
    bit = zeros(2,2,num_bits);
    
    %break up path into segments
    for i = 1 : num_segments
        
        segment(:,:,i) = Path_bez((i-1)*length(t)+1:length(t)*i,:);
        
    end
    
    %populate bit
    for i = 1 : num_bits
        
        bit(:,:,i) = Path_bez(i:i+1,:);
        
    end
    
    
    %calculate lengths of each segment
    for i = 1 : num_segments
        
        for j = 2 : length(t)
            segment_length(i) = segment_length(i) + norm ( segment(j,:,i) - segment(j-1,:,i));
        end
        
        %check
        if segment_length(i) < step_min
            segment_length(i) = step_min;
        end
        if segment_length(i) > step_max
            segment_length(i) = step_max;
        end
        
        
    end
    
    %calculate lengths (velocity, since /delta_t) of each bit
    for i = 1 : num_bits
        bit_length(i) = norm( bit(2,:,i) - bit(1,:,i))/delta_t;
        
        %check
        if bit_length(i) < step_min
            bit_length(i) = step_min;
        end
        if bit_length(i) > step_max
            bit_length(i) = step_max;
        end
    end
    
    
    
    %compare lengths to speed
    
    for i = 1 : num_bits
        
        color_var_b(i) = (bit_length(i)-step_min)/(step_max-step_min);
        
    end
    
    r_color_var_b = zeros(num_bits,1);
    g_color_var_b = zeros(num_bits,1);
    b_color_var_b = zeros(num_bits,1);
    
    %based on speed, change color
    for i = 1 : num_segments
        
        color_var(i) = (segment_length(i)-step_min)/(step_max-step_min);
        
    end
    
    
    
    c_r = color_r(color_var);
    c_g = color_g(color_var);
    c_b = color_b(color_var);
    
    
    %plot
    
    if d_speed_color == 1
        
        for i = 1 : num_bits
            
            plot(bit(1:2,1,i),bit(1:2,2,i),'Color',[cb*(color_var_b(i)),cb*(1-color_var_b(i)),0]);
            
        end
        
    else
        
        for i = 1 : num_segments
            
            plot(segment(:,1,i),segment(:,2,i),'Color',[cb*c_r(i), cb*c_g(i), cb*c_b(i)]);
            
        end
        
    end
    
    
    
else
    
    plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, cb, 0]); %plots path of UAV
    
end

if uav_finite_size == 0
    for i = 1 : length(path_start)
        
        if speed_color == 1
            
            plot(path_start(i,1),path_start(i,2),'o','Color',[cb*(color_var_b(i)),cb*(1-color_var_b(i)),0]);
            
        else
            
            plot(path_start(i,1),path_start(i,2),'og');
            
        end
        
    end
end

if uav_finite_size == 1
    for i = 1 : length(path_start)
        
        if speed_color == 1
            
            cs = 2*uav_ws/cx;
            x = path_start(i,1) - uav_ws : cs : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle
            
            plot(x,y,'Color',[cb*c_r(i), cb*c_g(i), cb*c_b(i)]);
            plot(x,y1,'Color',[cb*c_r(i), cb*c_g(i), cb*c_b(i)]);
            
        else
            cs = 2*uav_ws/cx;
            x = path_start(i,1) - uav_ws : cs : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle
            
            plot(x,y,'Color',[0, cb, 0]);
            plot(x,y1,'Color',[0, cb, 0]);
        end
    end
end

%plot landing area
cs = 2*lr/cx;
x = xf(1) - lr : cs : xf(1)+ lr;
y =  (lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %top part of circle
y1 = -(lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %bottom part of circle

plot(x,y,'g--');
plot(x,y1,'g--');

%plot segment of path from inside landing zone to final destination
plot([Path_bez(length(Path_bez),1) xf(1)],[Path_bez(length(Path_bez),2) xf(2)], 'Color',...
    [cb*c_r(length(c_r)), cb*c_g(length(c_g)), cb*c_b(length(c_b))] );

%-----------------------------------------%

for i = 1 : n_obs %-------- static obstacles ----------%
    
    
    plot(obs(i,1),obs(i,2),'xk'); % staic obstacles' centers
    cs = 2*obs_rad(i)/cx;
    x = obs(i,1) - obs_rad(i) : cs : obs(i,1)+ obs_rad(i);
    y =  (obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %top part of circle
    y1 = -(obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %bottom part of circle
    
    plot(x,y,'k');
    plot(x,y1,'k');
    
    
end  %--------------------------------------%

xlim([0 100]);
ylim([0 100]);
hold off


%compare paths created using various number of look ahead paths
if compare_num_path == 1
    
    if num_path == 1
        save('.\Compare\path_1.txt','Path_bez','-ascii');
        save('.\Compare\start_1.txt','path_start','-ascii');
    elseif num_path == 2
        save('.\Compare\path_2.txt','Path_bez','-ascii');
        save('.\Compare\start_2.txt','path_start','-ascii');
    elseif num_path == 3
        save('.\Compare\path_3.txt','Path_bez','-ascii');
        save('.\Compare\start_3.txt','path_start','-ascii');
    elseif num_path == 4
        save('.\Compare\path_4.txt','Path_bez','-ascii');
        save('.\Compare\start_4.txt','path_start','-ascii');
    elseif num_path == 5
        save('.\Compare\path_5.txt','Path_bez','-ascii');
        save('.\Compare\start_5.txt','path_start','-ascii');
    elseif num_path == 6
        save('.\Compare\path_6.txt','Path_bez','-ascii');
        save('.\Compare\start_6.txt','path_start','-ascii');
    end
    
end

%save path info to use in 'compare file'
if save_path == 1
    
    if optimize_energy_use == 1
        if one_path == 1
            save('.\Compare\path_e_opt.txt','Path_bez','-ascii');
            save('.\Compare\start_e_opt.txt','path_start','-ascii');
            
        else
            save('.\Compare\path_e.txt','Path_bez','-ascii');
            save('.\Compare\start_e.txt','path_start','-ascii');
        end
    elseif optimize_time == 1
        if one_path == 1
            save('.\Compare\path_t_opt.txt','Path_bez','-ascii');
            save('.\Compare\start_t_opt.txt','path_start','-ascii');
            
        else
            save('.\Compare\path_t.txt','Path_bez','-ascii');
            save('.\Compare\start_t.txt','path_start','-ascii');
        end
    else
        if one_path == 1
            save('.\Compare\path_d_opt.txt','Path_bez','-ascii');
            save('.\Compare\start_d_opt.txt','path_start','-ascii');
            
        else
            save('.\Compare\path_d.txt','Path_bez','-ascii');
            save('.\Compare\start_d.txt','path_start','-ascii');
        end
        
        
        
    end
end

%save guess to start one_path
if one_path == 0
    Bez_points = [Bez_points; x_fin];
    
    if optimize_time == 1
        
        %attempt to see if it can plan path with one less (note -2) segment
        %Bez_points_t = Bez_points(1:length(Bez_points)-2,:);
        
    end
    
end

%output of compare (energy, distance, time)
[td, tt, te] = compare_of(Bez_points,optimize_energy_use,optimize_time);

%profiling tools
profiling_info = profile('info');

toc