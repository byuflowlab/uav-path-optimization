% ------- Main File ------ %
% Author : Bryce Ingersoll
% Institution: Brigham Young University, FLOW Lab
% Last Revised : 7/6/16
% ------------------------ %

clear; clc; close all;

numlayouts = 1000;

feasiblepath = zeros(numlayouts,1);
timeelapsed = zeros(numlayouts,1);
te = zeros(numlayouts,1);
td =  zeros(numlayouts,1);
tt = zeros(numlayouts,1);

for z = 1 : numlayouts
    
    %print layout number
    z
    
%Add paths
addpath(genpath('.\Objective_Functions\'));
addpath(genpath('.\Constraints\'));
addpath(genpath('.\ColorPath\'));
addpath(genpath('.\Compare\'));
addpath(genpath('.\OptimalPathGuesses\'));
addpath(genpath('.\CalculateEnergyUse\'));

%profiling tools
%profile on

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
global max_speed min_speed D_eta_opt;
global l_l_last;

%------------Algorithm Options------------%
Dynamic_Obstacles = 0;

num_path = 3;              %Receding Horizon Approach (any number really, but 3 is standard)
ms_i = 5;                  %number of guesses for multi start (up to 8 for now, up to 3 for smart)
uav_finite_size = 1;       %input whether want to include UAV size
check_viability = 1;       %Exits if unable to find viable path

%Objective Function
optimize_energy_use = 1;    %changes which objective function is used
optimize_time =  0;         %if both are zero, then path length is optimized

max_func_evals = 10000;
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
save_path = 0;        %save path data to use in compare
sds = 0;              %Allows a closer view of dynamic obstacle avoidance
cx = 50;

create_video = 1;          %saves the solutions of the multistart approach at each iteration

% Gradient Calculation Options
obj_grad = 1;           %if this is 1 and below line is 0, complex step method will be used to calculate gradients
analytic_gradients = 1;
ag = analytic_gradients;

cons_grad = 1;          %if this is 1 and below line is 0, complex step method will be used to calculate gradients
analytic_constraint_gradients = 1;
acg = analytic_constraint_gradients;

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

%----------------plane geometry/info----------------%
%UAV parameter values
rho = 1.225; %air density
f = .2;   %equivalent parasite area
W = 10; %weight of aircraft
span = .20;   %span
eo = 0.9; %Oswald's efficiency factor

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

l = 0;

%parameterization vector t
global delta_t;
t = linspace(0,1,11);
delta_t = t(2) - t(1);


%for plot_both function
%global Path_bez;

turn_r = 5; %turn radius, m

%maximum/stall speed, m/s
max_speed = 15;
min_speed = 10;

l_l_last = (min_speed)/(length(t));

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
x_sp = [0,0];
x0 = x_sp;
xf = [100,100];
Bez_points = [];
lr = 15; %landing zone radius; should be =< 15
%--------------------------------------------------%

%-------static obstacle information---------%
%rng(3); %50/4/3
%rng(4); %49/4/3
%rng(59); %54/4/3 or 34/4/3
%rng(60); %50/4/3
%rng(13); %40/4/3
%rng(15); %40/4/3
%rng(20); %40/4/3
%rng(8)
rng(z);
n_obs = 35; %number of static obstacles
obs = rand(n_obs,2)*90+5; %obstacle locations
rng(4); %for partially random obstacle size
obs_rad = (3-uav_ws) +  rand(n_obs,1)*3; %obstacle radius
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
x_next = x_new;

% note: each iteration of while loop represents some time step, in which
% UAV travels on path and dynamic obstacles move

%fmincon options
if obj_grad == 1 && cons_grad == 1
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter,...
        'GradObj','on','GradCon','on','DerivativeCheck','off');
elseif obj_grad == 0 && cons_grad == 1
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter,...
        'GradObj','off','GradCon','on','DerivativeCheck','off');
elseif obj_grad == 1 && cons_grad == 0
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter,...
        'GradObj','on','GradCon','off','DerivativeCheck','off');
else
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',max_func_evals,'MaxIter',max_iter,...
        'GradObj','off','GradCon','off');
end

tic % begin optimization time

while ( ( (x_next(2*num_path,1)-xf(1))^2 + (x_next(2*num_path,2)-xf(2))^2 )^0.5 > lr )
    
    %record number of paths
    l = l + 1;
    
    
    for i = 1 : ms_i %multistart approach to find best solution
        
        %choose objective function
        if optimize_energy_use == 1
            
            
            [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_e, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options);
            
        elseif optimize_time == 1
            
            
            [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_t, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options);
            
        else
            
            
            [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_d, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options);
            
        end
        
%         %check curvature
%         c = check_curvature_new(i);
%         
%         %if constraints are violated, make infeasible
%         if any(c > 0)
%             %e(i,l) = -2;
%         end
    end
    
    for i = 1 : ms_i %calculate how good solutions are
        
        % For make_video
        if create_video == 1
            
            if i == 1
                solution1 = [solution1; x_new(:,:,i)];
            elseif i == 2
                solution2 = [solution2; x_new(:,:,i)];
            elseif i == 3
                solution3 = [solution3; x_new(:,:,i)];
            elseif i == 4
                solution4 = [solution4; x_new(:,:,i)];
            elseif i == 5
                solution5 = [solution5; x_new(:,:,i)];
            end
        end
        
        if optimize_energy_use == 1
            d_check(i) = opt_e(x_new(:,:,i));
            
        elseif optimize_time == 1
            d_check(i) = opt_t(x_new(:,:,i));
            
        else
            d_check(i) = opt_d(x_new(:,:,i));
            
        end
        
        %'remove' solutions that converged to an infeasible point
        
        if e(i,l) == -2
            
            d_check(i) = d_check(i)*10;
            
        end
        
        
    end
    
    %Check for viable paths
    check = (e == -2);
    if all(check(:,l)) == 1 && check_viability == 1
        %error('Unable to find viable path.');
        feasiblepath(z) = 1;
    end
    
    for i = 1 : ms_i %choose best solution, use for next part
        
        if d_check(i) == min(d_check)
            
            x_next = x_new(:,:,i);
            
        end
    end
    
    %
    initial = 0;
    
    % makes the path of the UAV for this section
    for i = 1 : length(t)
        
        path_part(i,:) = (1-t(i))^2*x0(1,:) + 2*(1-t(i))*t(i)*x_next(1,:)+t(i)^2*x_next(2,:);
        
%         if i > 1
%         norm(path_part(i,:)-path_part(i-1,:))
%         end
        
    end
    
    

    
    %make the planned path of the UAV
    if num_path > 1
        for j = 1 : (num_path-1)
            for i = 1 : length(t)
                path_planned(i+(j-1)*length(t),:) = (1-t(i))^2*x_next(2*j,:) + 2*(1-t(i))*t(i)*x_next(2*j+1,:)+t(i)^2*x_next(2*j+2,:);
            end
        end
    end
    
    if Show_Steps == 1
        
        plot_int_steps(l, square_axes, color_bar, totl, x_sp, cx, speed_color, path_part, path_planned, Path_bez, d_speed_color, cb...
            ,linewidth, radar, show_sp, show_end, sds);
    end
    
    %----------------------------------------------------------%
    
    %record where start of each path is
    path_start = [path_start; path_part(1,:)];
    
    %continues the path which will be plotted
    Path_bez = [Path_bez; path_part];
    
    l_l_last = norm(Path_bez(length(Path_bez),:)-Path_bez(length(Path_bez)-1,:));
    
    %set new starting point
    x0 = x_next(2,:);
    
    %set Pmid
    Pmid = x_next(1,:);
    
    %choose new guess for next iteration
    xi = multi_start(ms_i);
    
    %print current location
    %x_next(2,:)
    
    Bez_points = [Bez_points; x_next(1:2,:)];
    
end %while

timeelapsed(z) = toc; % end optimization time

Bez_points = [Bez_points; x_next(3:num_path*2,:)];

%Final Plot
% FinalPlot_robust_study(path_start, Path_bez, z, square_axes, totl, color_bar, speed_color...
%    , delta_t, d_speed_color, cb, cx, lr, x_sp);


%add planned path
Path_bez = [Path_bez; path_planned(2:length(path_planned),:)];

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
        
        save('.\Compare\path_e.txt','Path_bez','-ascii');
        save('.\Compare\start_e.txt','path_start','-ascii');
        
    elseif optimize_time == 1
        
        save('.\Compare\path_t.txt','Path_bez','-ascii');
        save('.\Compare\start_t.txt','path_start','-ascii');
        
    else
        save('.\Compare\path_d.txt','Path_bez','-ascii');
        save('.\Compare\start_d.txt','path_start','-ascii');
        
    end
end


%output of compare (energy, distance, time)
[td(z), tt(z), te(z)] = compare_of(Path_bez,Bez_points,optimize_energy_use,optimize_time);

%profiling tools
profiling_info = profile('info');

%toc % end optimization and plotting time

end

numberofinfeasiblepaths = sum(feasiblepath);

successpercentage = (numlayouts-numberofinfeasiblepaths)/numlayouts

avgtime = sum(timeelapsed)/numlayouts

avg_d = sum(td)/numlayouts

avg_t = sum(tt)/numlayouts

avg_e = sum(te)/numlayouts