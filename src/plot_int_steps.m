function [] = plot_int_steps(l, square_axes, color_bar, totl, x_sp, cx, speed_color, path_part, path_planned, Path_bez, d_speed_color, cb...
    ,linewidth, radar, show_sp, show_end)

%-------global variables----------%
global xf; %final position
global x0; %current starting pointPath_bez
global step_max; %max step distance
global step_min; %minimum step distance
global t; %parameterization variable
global n_obs; %number of obstacles
global obs; %positions of obstacles
global obs_rad; %radius of obstacles
global num_path; %number of segments optimized
global Dynamic_Obstacles;
global x_next; %used in multi_start function
global uav_ws; %UAV wing span
global initial; % to calculate d_l_min
initial = 1;
global uav_finite_size;
global delta_t

%--------------------------------------- Plot -------------------------------------%
    figure(l);
    hold on
    
    if square_axes == 1
        axis square
    end
    
    if color_bar == 1
        colorbar('southoutside','Ticks',[0,0.20,0.4,0.6,0.8,1],'TickLabels',{'V_{min}, 10 m/s','11 m/s','12 m/s','13 m/s','14 m/s','V_{max},15 m/s'},'fontsize',11);
    end
    
    if totl == 1
        
        set(gca,'XTickLabel','')
        set(gca,'YTickLabel','')
        
    end
    
    xlim([x_sp(1) xf(1)]);
    ylim([x_sp(2) xf(2)]);
    
    %pause
    
    if Dynamic_Obstacles == 0
        
        %-------------plot static obstacles-----------%
        for i = 1 : n_obs
            cs  = 2*obs_rad(i)/cx;
            plot(obs(i,1),obs(i,2),'xk'); % static obstacles' centers
            x = obs(i,1) - obs_rad(i) : cs : obs(i,1)+ obs_rad(i);
            y =  (obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %top part of circle
            y1 = -(obs_rad(i)^2 - (x - obs(i,1)).^2).^0.5 + obs(i,2); %bottom part of circle
            
            plot(x,y,'k');
            plot(x,y1,'k');
            
        end
        
    end
    
    %pause
    
    %-------------------UAV Path------------------------%
    
    %plot path already traversed as a normal line
    
    
    if speed_color == 1
        
        num_segments = (length(path_part)+length(path_planned)+length(Path_bez))/length(t);
        num_bits = (length(path_part)+length(path_planned)+length(Path_bez))-1;
        
        segment_length = zeros(num_segments,1);
        bit_length = zeros(num_bits,1);
        
        segment = zeros(length(t),2,num_segments);
        bit = zeros(2,2,num_bits);
        
        %break up path into segments
        path_int = [Path_bez; path_part; path_planned];
        
        for i = 1 : num_segments
            
            segment(:,:,i) = path_int((i-1)*length(t)+1:length(t)*i,:);
            
        end
        
        %populate bit
        for i = 1 : num_bits
            
            bit(:,:,i) = path_int(i:i+1,:);
            
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
                
                
                if i < length(t)*(l-1)
                    
                    %path already traveled
                    plot(bit(1:2,1,i),bit(1:2,2,i),'Color',[cb*(color_var_b(i)),cb*(1-color_var_b(i)),0]);
                end
                
                if i >= length(t)*(l-1) && i < length(t)*l
                    
                    %plot path that UAV has just traversed as a bold line
                    plot(bit(1:2,1,i),bit(1:2,2,i),'Color',[cb*(color_var_b(i)),cb*(1-color_var_b(i)),0],'LineWidth',linewidth);
                    
                else
                    
                    %plot path that UAV has planned as dashed line
                    if num_path > 1
                        plot(bit(1:2,1,i),bit(1:2,2,i),'--','Color',[cb*(color_var_b(i)),cb*(1-color_var_b(i)),0]);
                    end
                    
                end
                
            end
            
        else
            
            for i = 1 : num_segments
                if i <= l-1
                    plot(segment(:,1,i),segment(:,2,i),'Color',[cb*c_r(i),cb*c_g(i),cb*c_b(i)]);
                end
                
                if i > l-1 && i <= l
                    plot(segment(:,1,i),segment(:,2,i),'Color',[cb*c_r(i),cb*c_g(i),cb*c_b(i)],'LineWidth',linewidth);
                    
                else
                    plot(segment(:,1,i),segment(:,2,i),'--','Color',[cb*c_r(i),cb*c_g(i),cb*c_b(i)]);
                end
            end
            
        end
        
    else
        
        if l  == 1
            %nothing
        else
            plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, cb, 0]);
        end
        
        %plot path that UAV has just traversed as a bold line
        plot(path_part(:,1),path_part(:,2),'Color',[0, cb, 0],'LineWidth',2);
        
        %plot path that UAV has planned as dashed line
        if num_path > 1
            plot(path_planned(:,1),path_planned(:,2),'--','Color',[0, 0.5, 0]);
        end
        
    end
    
    
    
    %         %plot location of UAV on traversed line as circle for each time step
    %         for i = 1 : length(t)
    %             plot(path_part(i,1),path_part(i,2),'go');
    %         end
    
    
    %plot radar of UAV
    if radar == 1
        
        rl = num_path*max_speed;
        
        cs = 2*rl/cx;
        x = x0(1) - rl : cs : x0(1)+ rl;
        y =  (rl^2 - (x - x0(1)).^2).^0.5 + x0(2); %top part of circle
        y1 = -(rl^2 - (x - x0(1)).^2).^0.5 + x0(2); %bottom part of circle
        
        plot(x,y,'g');
        plot(x,y1,'g');
        
    end
    
    %plot UAV as circle at first and last time step
    
    if uav_finite_size == 1
        %plot where it is at start of time step
        cs = 2*uav_ws/cx;
        x = path_part(1,1) - uav_ws : cs : path_part(1,1)+ uav_ws;
        y =  (uav_ws^2 - (x - path_part(1,1)).^2).^0.5 + path_part(1,2); %top part of circle
        y1 = -(uav_ws^2 - (x - path_part(1,1)).^2).^0.5 + path_part(1,2); %bottom part of circle
        
        if speed_color == 1
            
            plot(x,y,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
            plot(x,y1,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
            
        else
            
            plot(x,y,'Color',[0, cb, 0]);
            plot(x,y1,'Color',[0, cb, 0]);
            
        end
        
        if show_sp == 1
            %plot where it is at start of time step
            cs = 2*uav_ws/cx;
            x = x_next(1,1) - uav_ws : cs : x_next(1,1)+ uav_ws;
            y =  (uav_ws^2 - (x - x_next(1,1)).^2).^0.5 + x_next(1,2); %top part of circle
            y1 = -(uav_ws^2 - (x - x_next(1,1)).^2).^0.5 + x_next(1,2); %bottom part of circle
            
            if speed_color == 1
                
                plot(x,y,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                plot(x,y1,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                
            else
                
                plot(x,y,'Color',[0, cb, 0]);
                plot(x,y1,'Color',[0, cb, 0]);
                
            end
            
            %plot where it is at start of next time step
            cs = 2*uav_ws/cx;
            x = x_next(3,1) - uav_ws : cs : x_next(3,1)+ uav_ws;
            y =  (uav_ws^2 - (x - x_next(3,1)).^2).^0.5 + x_next(3,2); %top part of circle
            y1 = -(uav_ws^2 - (x - x_next(3,1)).^2).^0.5 + x_next(3,2); %bottom part of circle
            
            if speed_color == 1
                
                plot(x,y,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                plot(x,y1,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                
            else
                
                plot(x,y,'Color',[0, cb, 0]);
                plot(x,y1,'Color',[0, cb, 0]);
                
            end
            
            %plot where it is at start of time step
            cs = 2*uav_ws/cx;
            x = x_next(5,1) - uav_ws : cs : x_next(5,1)+ uav_ws;
            y =  (uav_ws^2 - (x - x_next(5,1)).^2).^0.5 + x_next(5,2); %top part of circle
            y1 = -(uav_ws^2 - (x - x_next(5,1)).^2).^0.5 + x_next(5,2); %bottom part of circle
            
            if speed_color == 1
                
                plot(x,y,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                plot(x,y1,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                
            else
                
                plot(x,y,'Color',[0, cb, 0]);
                plot(x,y1,'Color',[0, cb, 0]);
                
            end
            
        end
        
        %plot where it is at end of time step
        %plot where it is at start of time step
        cs = 2*uav_ws/cx;
        x = path_part(length(t),1) - uav_ws : cs : path_part(length(t),1)+ uav_ws;
        y =  (uav_ws^2 - (x - path_part(length(t),1)).^2).^0.5 + path_part(length(t),2); %top part of circle
        y1 = -(uav_ws^2 - (x - path_part(length(t),1)).^2).^0.5 + path_part(length(t),2); %bottom part of circle
        
        if speed_color == 1
            
            plot(x,y,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
            plot(x,y1,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
            
        else
            
            plot(x,y,'Color',[0, cb, 0]);
            plot(x,y1,'Color',[0, cb, 0]);
            
        end
    end
    
    %plot UAV as circle at last time step for future planned path
    if uav_finite_size == 1
        if num_path > 1
            for j = 1 : (num_path-1)
                %plot where it is at end of time step
                cs = 2*uav_ws/cx;
                x = path_planned(j*length(t),1) - uav_ws : cs : path_planned(j*length(t),1)+ uav_ws;
                y =  (uav_ws^2 - (x - path_planned(j*length(t),1)).^2).^0.5 + path_planned(j*length(t),2); %top part of circle
                y1 = -(uav_ws^2 - (x - path_planned(j*length(t),1)).^2).^0.5 + path_planned(j*length(t),2); %bottom part of circle
                
                if speed_color == 1
                    
                    plot(x,y,'Color',[cb*c_r(j+l), cb*c_g(j+l), cb*c_b(j+l)]);
                    plot(x,y1,'Color',[cb*c_r(j+l), cb*c_g(j+l), cb*c_b(j+l)]);
                    
                else
                    
                    plot(x,y,'Color',[0, cb, 0]);
                    plot(x,y1,'Color',[0, cb, 0]);
                    
                end
            end
        end
    end
    
    if Dynamic_Obstacles == 1
        
        if sds == 1 && (l == 7 || l == 8)
            
            for i = 1 : length(t)
                
                %change figure number
                figurenum = l*20 + i;
                figure(figurenum);
                hold on
                %set plot boundary
                xlim([40 60]);
                ylim([40 60]);
                timestep = l + t(i);
                xlabel(['Time Step = ' num2str(timestep) ' s'])
                %plot dynamic obstacles
                
                %plot small square at center of dynamic obstacles at each time step
                for k = 1 : n_obsd
                    
                    plot(obs_d_v(k,1)*t(i) + obs_d_cp(k,1),obs_d_v(k,2)*t(i) + obs_d_cp(k,2),'s','Color',[0.5,0,0]);
                    
                    odh = obs_d_cp; % to make it easier to type
                    cs = 2*obs_d_s(k)/cx;
                    x = obs_d_v(k,1)*t(i) + odh(k,1) - obs_d_s(k) : cs : obs_d_v(k,1)*t(i) + odh(k,1)+ obs_d_s(k);
                    y =  (obs_d_s(k)^2 - (x - (obs_d_v(k,1)*t(i)+odh(k,1))).^2).^0.5 + odh(k,2) + obs_d_v(k,2)*t(i); %top part of circle
                    y1 = -(obs_d_s(k)^2 - (x - (obs_d_v(k,1)*t(i)+odh(k,1))).^2).^0.5 + odh(k,2) + obs_d_v(k,2)*t(i); %bottom part of circle
                    
                    plot(x,y,'Color',[0.5,0,0],'LineWidth',2);
                    plot(x,y1,'Color',[0.5,0,0],'LineWidth',2);
                end
                %plot position of UAV
                cs = 2*uav_ws(k)/cx;
                x = segment(i,1,l) - uav_ws : cs : segment(i,1,l)+ uav_ws;
                y =  (uav_ws^2 - (x - segment(i,1,l)).^2).^0.5 + segment(i,2,l); %top part of circle
                y1 = -(uav_ws^2 - (x - segment(i,1,l)).^2).^0.5 + segment(i,2,l); %bottom part of circle
                plot(x,y,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                plot(x,y1,'Color',[cb*c_r(l), cb*c_g(l), cb*c_b(l)]);
                
                hold off
            end
        end
        
        
        
        
        %plot small square at center of dynamic obstacles at each time step
        for k = 1 : n_obsd
            for i = 1 : length(t)
                plot(obs_d_v(k,1)*t(i) + obs_d_cp(k,1),obs_d_v(k,2)*t(i) + obs_d_cp(k,2),'s','Color',[0.5,0,0]);
            end
        end
        %plot most recent previous placement of dynamic obstacles as bold circles
        for k = 1 : n_obsd
            
            plot(obs_d_cp(k,1),obs_d_cp(k,2),'s','Color',[0.5,0,0]); %plot center of obstacles
            odh = obs_d_cp; % to make it easier to type
            
            cs = 2*obs_d_s(k)/cx;
            x = odh(k,1) - obs_d_s(k) : cs : odh(k,1)+ obs_d_s(k);
            y =  (obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %top part of circle
            y1 = -(obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %bottom part of circle
            
            plot(x,y,'Color',[0.5,0,0],'LineWidth',2);
            plot(x,y1,'Color',[0.5,0,0],'LineWidth',2);
        end
        
        
        %plot current position of dynamic obstacles as dashed bold circles
        %dynamic obstacles position update
        for k = 1 : n_obsd
            obs_d_cp(k,:) = obs_d_v(k,:) + obs_d_cp(k,:);
        end
        
        for k = 1 : n_obsd
            
            plot(obs_d_cp(k,1),obs_d_cp(k,2),'s','Color',[0.5,0,0]); %plot center of obstacles
            odh = obs_d_cp; % to make it easier to type
            
            cs = 2*obs_d_s(k)/cx;
            x = odh(k,1) - obs_d_s(k) : cs : odh(k,1)+ obs_d_s(k);
            y =  (obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %top part of circle
            y1 = -(obs_d_s(k)^2 - (x - odh(k,1)).^2).^0.5 + odh(k,2); %bottom part of circle
            
            plot(x,y,'--','Color',[0.5,0,0]);
            plot(x,y1,'--','Color',[0.5,0,0]);
        end
        
    end
    
    %show end
    if show_end == 1
        plot([path_planned(length(t)*(num_path-1),1) 100],[path_planned(length(t)*(num_path-1),2) 100],'Color',[0 0 0],'LineWidth',2);
    end
    
    hold off
    
end