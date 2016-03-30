clear; clc; close all;
tic
rng(1); %54/4/3 -pretty easy; 50/4/3 - more interesting
%rng(1); %40, 3.5/3; 50,0 and 2,7 - goes below obstacles
%rng(1); %49/4/3 %similar 3 comparison (2)
%rng(2); %50,3.75,3 - works for num_path = 3;
%rng(2); %52,3.5,3 -easy start, difficult middle portion
%rng(3); %50,4,3
%rng(4); %49,4,3
%rng(5); %40, 3/3; 50,0 and 2,6.5 - goes above
%rng(5); %49/4/3 - similar 3 comparison (1)
%rng(6); %50,4,3 - use for comparison of removing unconverged solutions
%rng(7); %50,4,3 %use this for opt_d, delta_t = 0.1, for num_path comparison
%rng(8); %54, 4, 3 %good example for ms=1 fails ms=3 succeeds
%rng(9); %50,4,3 - num_path comparison
%rng(11); %55,4,3 - used for the three comparison
%rng(18);  %4,3,54
%rng(22); %50/5/3 - difficult start, straight forward finish; for delta_t=0.2, fails if you don't remove bad solutions
%rng(51); %3.5,3,50 - optimized finish comparison
%rng(59); %3,3, 55   - use for comparison of three objective functions
%rng(59); %4,3,54
%rng(60); %3.5, 3, 40 or 4/3/50
%rng(101); %50/54,4,3 -difficult start, works for delta_t = 0.1

%-------ucur results-----------%

%first static obstacle avoidance (show steps)
%rng(2); %52,3.5,3,delta_t=0.1,ms=3,num_path=3

%additional static obstacle avoidance (don't show steps)
%rng(6); %50,4,3 - static2
%rng(18);  %4,3,54 - static3
%rng(59); %54/4/3 - static4, all of these are optimize_time

%dynamic obstacle avoidance
%rng(2); %3,3.5,1; dyn_case = 1; optimize time

%dynamic obstacle avoidance comparison
%rng(2); %3,3.5,1; dyn_case = 5; delta_t = 0.1

%---------paper results----------%

%SDOA
%rng(2); %44/4/3, dyn_case = 7

%Methodology
%rng(2); %50/4/3

%multi start approach
%rng(7); %50/4/3; figure(4), ms_i = 1

% EU vs. Time
%rng(60); %50/4/3 % 1
%rng(59); %54/4/3 % 2

%------------------------------%
% Bryce Ingersoll
%
%
%------------------------------%

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
global uav_finite_size;

%to calculate d_l_min
initial = 1;

%Plane parameters are found in optimize_fe function

%------------Algorithm Options------------%
Optimized_Finish = 1;
Dynamic_Obstacles = 0;
compare_num_path = 0;
num_path = 3;              %Receding Horizon Approach (any number really, but 3 is standard)
ms_i = 3;                  %number of guesses for multi start (up to 8 for now, up to 3 for smart)
uav_finite_size = 1;       %input whether want to include UAV size
optimize_energy_use = 0;    %changes which objective function is used
optimize_time = 0;          %if both are zero, then pathlength is optimized
final_plot = 1;
Show_Steps = 0;            %needs to be turned on when Dynamic_Obstacles is turned on
create_movie = 0;
save_path = 0;           %save path data to use in compare
remove_infeasible_sol = 1;

%----------------------------------------%

%-------------- one_path -----------------%
%plan entire path
% to run this, first need to run using 3-4 num_path, save that path, and
% use that as your initial guess; also need to change number of num_path to
% match what was previously solved for
one_path = 0; %need to make sure num_path is sufficiently high; if this is on, need to set ms_i = 1

%{
rng(8); %54/4/3
if one_path == 1
    num_path = 16;
    ms_i = 1;
    get_bez_points = @rng8;
end
%}


%{
rng(59); %4,3,54
if one_path == 1
    num_path = 16;
    ms_i = 1;
    get_bez_points = @rng59;
end
%}


l = 0;

%parameterization vector t
global delta_t;
delta_t = 0.1;
t = 0 : delta_t : 1;

%for plot_both function
%global Path_bez;

%----------------plane geometry/info----------------%
turn_r = 5; %turn radius

%maximum/stall speed (m/s) ?
max_speed = 15;
min_speed = 10;


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
%--------------------------------------------------%

%transalte UAV information to fit with algorithm
step_max = max_speed; %/2;
step_min = min_speed; %/2;

%-------static obstacle information--------%
n_obs = 50; %number of static obstacles
obs = rand(n_obs,2)*90+5; %obstacle locations
rng(4); %for partially random obstacle size
obs_rad = (4-uav_ws) +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%

%------dynamic obstacle information---------%
if Dynamic_Obstacles == 1
    
    global n_obsd obs_d_sp obs_d_v obs_d_s obs_d_cp;
    
    %choose 1-4 for cases (see function for description)
    [n_obsd, obs_d_sp, obs_d_s, obs_d_v]  = dyn_case(7);
    
    obs_d_s = obs_d_s-ones(n_obsd,1)*uav_ws; %size of obstacles, also used (5)
    obs_d_cp = obs_d_sp; %current position of obstacles
    obs_d_cp_hist(1,:,1) = obs_d_sp(1,:);
end
%-------------------------------------------%

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
Pmid = [-3,-3];

Path_bez = [0,0];

path_start = [];

%initial guess(es)
start = 0;

xi = multi_start(ms_i);

%start
start = 1;

%x_new is not close to final position
x_new = zeros(2*num_path,2);


% note: each iteration of while loop represents some time step, in which
% UAV travels on path and dynamic obstacles move


while ( abs(x_new(2*num_path,1)-xf(1)) > 10^0 ) && ( abs(x_new(2*num_path,2)-xf(2)) > 10^0 )
    
    if one_path == 1
        break;
    end
    %record number of paths
    l = l + 1;
    
    
    for i = 1 : ms_i %multistart approach to find best solution
        
        %choose objective function
        if optimize_energy_use == 1
            
            options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',500000,'MaxIter',100000);
            [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_e, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options);
            
        elseif optimize_time == 1
            
            options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',500000,'MaxIter',100000);
            [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_t, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options);
            
        else
            
            %options = optimoptions('fmincon','Algorithm','sqp','GradObj','on');
            options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',500000,'MaxIter',100000);
            [x_new(:,:,i),~,e(i,l)] = fmincon(@opt_d, xi(:,:,i) , A, b, Aeq, beq, lb, ub, @cons,options);
            
        end
        
    end
    
    for i = 1 : ms_i %calculate how good solutions are
        
        if optimize_energy_use == 1
            d_check(i) = opt_e(x_new(:,:,i));
            
        elseif optimize_time == 1
            d_check(i) = opt_t(x_new(:,:,i));
            
        else
            d_check(i) = opt_d(x_new(:,:,i));
            
        end
        
        %'remove' solutions that converged to an infeasible point
        if remove_infeasible_sol == 1
            if e(i,l) == -2
                
                d_check(i) = d_check(i)*10;
                
            end
        end
        
    end
    
    for i = 1 : ms_i %choose best solution, use for next part
        
        if d_check(i) == min(d_check)
            
            x_next = x_new(:,:,i);
            
        end
    end
    
    %
    initial = 0;
    
    %switch to last optimizing function
    if abs(x_next(2*num_path,1)-xf (1)) < 10^-1  && abs(x_next(2*num_path,2)-xf (2)) < 10^-1 && Optimized_Finish == 1
        break
    end
    
    % makes the path of the UAV for this section
    for i = 1 : length(t)
        
        path_part(i,:) = (1-t(i))^2*x0(1,:) + 2*(1-t(i))*t(i)*x_next(1,:)+t(i)^2*x_next(2,:);
        
    end
    
    %make the planned path of the UAV
    if num_path > 1
        for j = 1 : (num_path-1)
            for i = 1 : length(t)
                path_planned(i+(j-1)*length(t),:) = (1-t(i))^2*x_next(2*j,:) + 2*(1-t(i))*t(i)*x_next(2*j+1,:)+t(i)^2*x_next(2*j+2,:);
            end
        end
    end
    
    %--------------------------------------- Plot -------------------------------------%
    if Show_Steps == 1
        figure(l);
        hold on
        
        %-------------------UAV Path------------------------%
        
        %plot path already traversed as a normal line
        plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, 0.5, 0]);
        
        %plot path that UAV has just traversed as a bold line
        plot(path_part(:,1),path_part(:,2),'Color',[0, 0.5, 0],'LineWidth',2);
        
        %plot path that UAV has planned as dashed line
        if num_path > 1
            plot(path_planned(:,1),path_planned(:,2),'--','Color',[0, 0.5, 0]);
        end
        
        %         %plot location of UAV on traversed line as circle for each time step
        %         for i = 1 : length(t)
        %             plot(path_part(i,1),path_part(i,2),'go');
        %         end
        
        %plot UAV as circle at first and last time step
        if uav_finite_size == 1
            %plot where it is at start of time step
            x = path_part(1,1) - uav_ws : 0.001 : path_part(1,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_part(1,1)).^2).^0.5 + path_part(1,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_part(1,1)).^2).^0.5 + path_part(1,2); %bottom part of circle
            
            plot(x,y,'Color',[0, 0.5, 0]);
            plot(x,y1,'Color',[0, 0.5, 0]);
            
            %plot where it is at end of time step
            %plot where it is at start of time step
            x = path_part(length(t),1) - uav_ws : 0.001 : path_part(length(t),1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_part(length(t),1)).^2).^0.5 + path_part(length(t),2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_part(length(t),1)).^2).^0.5 + path_part(length(t),2); %bottom part of circle
            
            plot(x,y,'Color',[0, 0.5, 0]);
            plot(x,y1,'Color',[0, 0.5, 0]);
        end
        
        %plot UAV as circle at last time step for future planned path
        if uav_finite_size == 1
            if num_path > 1
                for j = 1 : (num_path-1)
                    %plot where it is at end of time step
                    x = path_planned(j*length(t),1) - uav_ws : 0.001 : path_planned(j*length(t),1)+ uav_ws;
                    y =  (uav_ws^2 - (x - path_planned(j*length(t),1)).^2).^0.5 + path_planned(j*length(t),2); %top part of circle
                    y1 = -(uav_ws^2 - (x - path_planned(j*length(t),1)).^2).^0.5 + path_planned(j*length(t),2); %bottom part of circle
                    
                    plot(x,y,'Color',[0, 0.5, 0]);
                    plot(x,y1,'Color',[0, 0.5, 0]);
                    
                end
            end
        end
        
        
        %-------------plot static obstacles-----------%
        for i = 1 : n_obs
            
            plot(obs(i,1),obs(i,2),'xb'); % static obstacles' centers
            x = obs(i,1) - obs_rad(i) : 0.001 : obs(i,1)+ obs_rad(i);
            y =  (obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %top part of circle
            y1 = -(obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %bottom part of circle
            
            plot(x,y,'b');
            plot(x,y1,'b');
            
        end
        
        if Dynamic_Obstacles == 1
            
            %plot small square at center of dynamic obstacles at each time step
            for k = 1 : n_obsd
                for i = 1 : length(t)
                    plot(obs_d_v(k,1)*t(i) + obs_d_cp(k,1),obs_d_v(k,2)*t(i) + obs_d_cp(k,2),'rs');
                end
            end
            %plot most recent previous placement of dynamic obstacles as bold circles
            for k = 1 : n_obsd
                
                plot(obs_d_cp(k,1),obs_d_cp(k,2),'rs'); %plot center of obstacles
                odh = obs_d_cp; % to make it easier to type
                
                x = odh(k,1) - obs_d_s(k) : 0.001 : odh(k,1)+ obs_d_s(k);
                y =  (obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %top part of circle
                y1 = -(obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %bottom part of circle
                
                plot(x,y,'r','LineWidth',2);
                plot(x,y1,'r','LineWidth',2);
            end
            
            
            %plot current position of dynamic obstacles as dashed bold circles
            %dynamic obstacles position update
            for k = 1 : n_obsd
                obs_d_cp(k,:) = obs_d_v(k,:) + obs_d_cp(k,:);
            end
            
            for k = 1 : n_obsd
                
                plot(obs_d_cp(k,1),obs_d_cp(k,2),'rs'); %plot center of obstacles
                odh = obs_d_cp; % to make it easier to type
                
                x = odh(k,1) - obs_d_s(k) : 0.001 : odh(k,1)+ obs_d_s(k);
                y =  (obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %top part of circle
                y1 = -(obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %bottom part of circle
                
                plot(x,y,'r--');
                plot(x,y1,'r--');
            end
            
        end
        
        xlim([0 100]);
        ylim([0 100]);
        hold off
        
    end
    %----------------------------------------------------------%
    
    %record where start of each path is
    path_start = [path_start; path_part(1,:)];
    
    %continues the path which will be plotted
    Path_bez = [Path_bez; path_part];
    
    %set new starting point
    x0 = x_next(2,:);
    
    %set Pmid
    Pmid = x_next(1,:);
    
    %choose new guess for next iteration
    xi = multi_start(ms_i);
    
    %print current location
    x_next(2,:)
    
    Bez_points = [Bez_points; x_next(1:2,:)];
    
end %while


%-------------------------final optimization------------------%
%final guess
if one_path == 1
    x_guess_final = get_bez_points();
else
    x_guess_final = multi_start(ms_i);
end



for i = 1 : ms_i %multistart approach to find best solution
    
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',500000,'MaxIter',100000);
    x_final(:,:,i) = fmincon(@final_dist, x_guess_final(:,:,i) , A, b, Aeq, beq, lb, ub, @final_con,options);
    
end

for i = 1 : ms_i %calculate how good solutions are
    
    d_check(i) = final_dist(x_final(:,:,i));
    
end

for i = 1 : ms_i %choose best solution, use for next part
    
    if d_check(i) == min(d_check)
        
        x_fin = x_final(:,:,i);
        
    end
end


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

if final_plot == 1
    figure(l+1);
    hold on
    
    %----------------plot UAV-------------------%
    
    plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, 0.5, 0]); %plots path of UAV
    
    if uav_finite_size == 0
        for i = 1 : length(path_start)
            plot(path_start(i,1),path_start(i,2),'og');
        end
    end
    
    if uav_finite_size == 1
        for i = 1 : length(path_start)
            x = path_start(i,1) - uav_ws : 0.001 : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle
            
            plot(x,y,'Color',[0, 0.5, 0]);
            plot(x,y1,'Color',[0, 0.5, 0]);
        end
    end
    
    
    %-----------------------------------------%
    
    for i = 1 : n_obs %-------- static obstacles ----------%
        
        
        plot(obs(i,1),obs(i,2),'xb'); % staic obstacles' centers
        x = obs(i,1) - obs_rad(i) : 0.001 : obs(i,1)+ obs_rad(i);
        y =  (obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %top part of circle
        y1 = -(obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %bottom part of circle
        
        plot(x,y,'b');
        plot(x,y1,'b');
        
        
    end  %--------------------------------------%
    
    % %--------- dynamic obstacles --------%
    % for k = 1 : n_obsd
    %     for i = 1 : length(obs_d_cp_hist);
    %         plot(obs_d_cp_hist(i,1),obs_d_cp_hist(i,2),'rs'); %plot center of obstacles
    %         odh = obs_d_cp_hist; % to make it easier to type
    %
    %         x = odh(i,1,k) - obs_d_s(k) : 0.001 : odh(i,1,k)+ obs_d_s(k);
    %         y =  (obs_d_s(k)^2 - (x - odh(i,1,k)).^2).^0.5 + odh(i,2,k); %top part of circle
    %         y1 = -(obs_d_s(k)^2 - (x - odh(i,1,k)).^2).^0.5 + odh(i,2,k); %bottom part of circle
    %
    %         plot(x,y,'r');
    %         plot(x,y1,'r');
    %     end
    % end
    
    xlim([0 100]);
    ylim([0 100]);
    hold off
end

%----Create video file----%
if create_movie == 1
    M = make_movie(Path_bez);
    movie2avi(M,'newmovie.avi','compression','none','fps',4);
end

%--- Evaluate Solution ---%
A_total_distance = evaluate_solution(Path_bez);

%compare paths created using various number of look ahead paths
if compare_num_path == 1
    
    if num_path == 1
        save('path_1.txt','Path_bez','-ascii');
        save('start_1.txt','path_start','-ascii');
    elseif num_path == 2
        save('path_2.txt','Path_bez','-ascii');
        save('start_2.txt','path_start','-ascii');
    elseif num_path == 3
        save('path_3.txt','Path_bez','-ascii');
        save('start_3.txt','path_start','-ascii');
    elseif num_path == 4
        save('path_4.txt','Path_bez','-ascii');
        save('start_4.txt','path_start','-ascii');
    elseif num_path == 5
        save('path_5.txt','Path_bez','-ascii');
        save('start_5.txt','path_start','-ascii');
    elseif num_path == 6
        save('path_6.txt','Path_bez','-ascii');
        save('start_6.txt','path_start','-ascii');
    end
    
end

%save path info to use in 'compare file'
if save_path == 1
    
    if optimize_energy_use == 1
        save('path_e.txt','Path_bez','-ascii');
        save('start_e.txt','path_start','-ascii');
    elseif optimize_time == 1
        save('path_t.txt','Path_bez','-ascii');
        save('start_t.txt','path_start','-ascii');
    else
        save('path_d.txt','Path_bez','-ascii');
        save('start_d.txt','path_start','-ascii');
    end
    
end


%save guess to start one_path
if one_path == 0
    Bez_points = [Bez_points; x_fin];
end

toc