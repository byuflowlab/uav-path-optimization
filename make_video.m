clear; clc; close all;
close all

%function [] = make_video(solution1 - solution5,
%This code is pretty hard-coded for ms_i = 5, num_path = 3; need to change

%information -- start
%inputs

%choose function
get_info = @make_video_5_20a;

[Bez_points, num_path, num_segments, delta_t, t, solution1, solution2, solution3, solution4, solution5,...
    Path_bez, ms_i, uav_ws, n_obs, obs, obs_rad] = get_info();

%-- information end --%

Bez_points = [0,0; Bez_points];

for i = 1 : num_segments-2
    
    bold_path1 = [];
    dashed_path1 = [];
    
    
    for k = 1 : ms_i+2
        
        fig_num = (i-1)*(ms_i+2)+k;
        
        figure(fig_num)
        
        hold on
        
        %plot previously traversed path
        if i == 1
            %don't plot
        else
            plot(Path_bez(1:(i-1)*length(t),1),Path_bez(1:(i-1)*length(t),2),'Color',[0, 0.5, 0]);
        end
        
        %plot solutions
        if k == 1
            solution = solution1;
        end
        if k == 2
            solution = solution2;
        end
        if k == 3
            solution = solution3;
        end
        if k == 4
           solution = solution4;
        end
        if k == 5
           solution = solution5;
        end
        if k == ms_i+1
            solution = Bez_points;
        end
        if k == ms_i+2
            solution = Bez_points;
        end
        
        %plot path that UAV has just traversed as a bold line
        for l = 1 : length(t)
            if k == ms_i+1
                bold_path1(l,:) = (1-t(l))^2*Bez_points(2*i-1,:) +...
                    2*(1-t(l))*t(l)*Bez_points(2*i,:)+t(l)^2*Bez_points(2*i+1,:);
            elseif k == ms_i+2
                bold_path1(l,:) = (1-t(l))^2*Bez_points(2*i-1,:) +...
                    2*(1-t(l))*t(l)*Bez_points(2*i,:)+t(l)^2*Bez_points(2*i+1,:);
            else
                bold_path1(l,:) = (1-t(l))^2*Bez_points(2*i-1,:) +...
                    2*(1-t(l))*t(l)*solution(1+2*(i-1)*num_path,:)+t(l)^2*solution(2+2*(i-1)*num_path,:);
            end
        end
        
        if k == ms_i+1 || k == ms_i+2
            
            plot(bold_path1(:,1),bold_path1(:,2),'Color',[0, 0.5, 0],'LineWidth',1);
            
        else
            
            plot(bold_path1(:,1),bold_path1(:,2),'Color',[0, 0.5, 0],'LineWidth',2);
            
        end
        
        %plot path that UAV has planned as dashed line
        for j = 1 : (num_path-1)
            for l = 1 : length(t)
                if k == ms_i+1
                    dashed_path1(l+(j-1)*length(t),:) = (1-t(l))^2*Bez_points(2*i+2*j-1,:) +...
                        2*(1-t(l))*t(l)*Bez_points(2*i+2*j,:)+t(l)^2*Bez_points(2*i+2*j+1,:);
                elseif k == ms_i+2
                    dashed_path1(l+(j-1)*length(t),:) = (1-t(l))^2*Bez_points(2*i+2*j-1,:) +...
                        2*(1-t(l))*t(l)*Bez_points(2*i+2*j,:)+t(l)^2*Bez_points(2*i+2*j+1,:);
                else
                    
                    dashed_path1(l+(j-1)*length(t),:) = (1-t(l))^2*solution(2*j+2*(i-1)*num_path,:)...
                        + 2*(1-t(l))*t(l)*solution(2*j+1+2*(i-1)*num_path,:)+t(l)^2*solution(2*j+2+2*(i-1)*num_path,:);
                end
            end
        end
        plot(dashed_path1(:,1),dashed_path1(:,2),'--','Color',[0, 0.5, 0])
        
        %plot uav positions
        for l = 1 : num_path
            
            if k == ms_i+1
                
                x = solution(2*i+2*l-1,1) - uav_ws : 0.001 : solution(2*i+2*l-1,1)+ uav_ws;
                y =  (uav_ws^2 - (x - solution(2*i+2*l-1,1)).^2).^0.5 + solution(2*i+2*l-1,2); %top part of circle
                y1 = -(uav_ws^2 - (x - solution(2*i+2*l-1,1)).^2).^0.5 + solution(2*i+2*l-1,2); %bottom part of circle
                
            elseif k == ms_i+2
                
                x = solution(2*i+2*l-1,1) - uav_ws : 0.001 : solution(2*i+2*l-1,1)+ uav_ws;
                y =  (uav_ws^2 - (x - solution(2*i+2*l-1,1)).^2).^0.5 + solution(2*i+2*l-1,2); %top part of circle
                y1 = -(uav_ws^2 - (x - solution(2*i+2*l-1,1)).^2).^0.5 + solution(2*i+2*l-1,2); %bottom part of circle
                
            else
                
                x = solution(6*i+2*l-6,1) - uav_ws : 0.001 : solution(6*i+2*l-6,1)+ uav_ws;
                y =  (uav_ws^2 - (x - solution(6*i+2*l-6,1)).^2).^0.5 + solution(6*i+2*l-6,2); %top part of circle
                y1 = -(uav_ws^2 - (x - solution(6*i+2*l-6,1)).^2).^0.5 + solution(6*i+2*l-6,2); %bottom part of circle
                
            end
            plot(x,y,'Color',[0, 0.5, 0]);
            plot(x,y1,'Color',[0, 0.5, 0]);
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
        
        
        %all images show static obstacles
        % each solution from multiple guesses at every iteration also should be
        % shown
        
        %save figure as jpg
        if fig_num == 1
            saveas(figure(fig_num),'pic1.jpg')
        elseif fig_num == 2
            saveas(figure(fig_num),'pic2.jpg')
        elseif fig_num == 3
            saveas(figure(fig_num),'pic3.jpg')
        elseif fig_num == 4
            saveas(figure(fig_num),'pic4.jpg')
        elseif fig_num == 5
            saveas(figure(fig_num),'pic5.jpg')
        elseif fig_num == 6
            saveas(figure(fig_num),'pic6.jpg')
        elseif fig_num == 7
            saveas(figure(fig_num),'pic7.jpg')
        elseif fig_num == 8
            saveas(figure(fig_num),'pic8.jpg')
        elseif fig_num == 9
            saveas(figure(fig_num),'pic9.jpg')
        elseif fig_num == 10
            saveas(figure(fig_num),'pic10.jpg')
        elseif fig_num == 11
            saveas(figure(fig_num),'pic11.jpg')
        elseif fig_num == 12
            saveas(figure(fig_num),'pic12.jpg')
        elseif fig_num == 13
            saveas(figure(fig_num),'pic13.jpg')
        elseif fig_num == 14
            saveas(figure(fig_num),'pic14.jpg')
        elseif fig_num == 15
            saveas(figure(fig_num),'pic15.jpg')
        elseif fig_num == 16
            saveas(figure(fig_num),'pic16.jpg')
        elseif fig_num == 17
            saveas(figure(fig_num),'pic17.jpg')
        elseif fig_num == 18
            saveas(figure(fig_num),'pic18.jpg')
        elseif fig_num == 19
            saveas(figure(fig_num),'pic19.jpg')
        elseif fig_num == 20
            saveas(figure(fig_num),'pic20.jpg')
        elseif fig_num == 21
            saveas(figure(fig_num),'pic21.jpg')
        elseif fig_num == 22
            saveas(figure(fig_num),'pic22.jpg')
        elseif fig_num == 23
            saveas(figure(fig_num),'pic23.jpg')
        elseif fig_num == 24
            saveas(figure(fig_num),'pic24.jpg')
        elseif fig_num == 25
            saveas(figure(fig_num),'pic25.jpg')
        elseif fig_num == 26
            saveas(figure(fig_num),'pic26.jpg')
        elseif fig_num == 27
            saveas(figure(fig_num),'pic27.jpg')
        elseif fig_num == 28
            saveas(figure(fig_num),'pic28.jpg')
        elseif fig_num == 29
            saveas(figure(fig_num),'pic29.jpg')
        elseif fig_num == 30
            saveas(figure(fig_num),'pic30.jpg')
        elseif fig_num == 31
            saveas(figure(fig_num),'pic31.jpg')
        elseif fig_num == 32
            saveas(figure(fig_num),'pic32.jpg')
        elseif fig_num == 33
            saveas(figure(fig_num),'pic33.jpg')
        elseif fig_num == 34
            saveas(figure(fig_num),'pic34.jpg')
        elseif fig_num == 35
            saveas(figure(fig_num),'pic35.jpg')
        elseif fig_num == 36
            saveas(figure(fig_num),'pic36.jpg')
        elseif fig_num == 37
            saveas(figure(fig_num),'pic37.jpg')
        elseif fig_num == 38
            saveas(figure(fig_num),'pic38.jpg')
        elseif fig_num == 39
            saveas(figure(fig_num),'pic39.jpg')
        elseif fig_num == 40
            saveas(figure(fig_num),'pic40.jpg')
        elseif fig_num == 41
            saveas(figure(fig_num),'pic41.jpg')
        elseif fig_num == 42
            saveas(figure(fig_num),'pic42.jpg')
        elseif fig_num == 43
            saveas(figure(fig_num),'pic43.jpg')
        elseif fig_num == 44
            saveas(figure(fig_num),'pic44.jpg')
        elseif fig_num == 45
            saveas(figure(fig_num),'pic45.jpg')
        elseif fig_num == 46
            saveas(figure(fig_num),'pic46.jpg')
        elseif fig_num == 47
            saveas(figure(fig_num),'pic47.jpg')
        elseif fig_num == 48
            saveas(figure(fig_num),'pic48.jpg')
        elseif fig_num == 49
            saveas(figure(fig_num),'pic49.jpg')
        elseif fig_num == 50
            saveas(figure(fig_num),'pic50.jpg')
        elseif fig_num == 51
            saveas(figure(fig_num),'pic51.jpg')
        elseif fig_num == 52
            saveas(figure(fig_num),'pic52.jpg')
        elseif fig_num == 53
            saveas(figure(fig_num),'pic53.jpg')
        elseif fig_num == 54
            saveas(figure(fig_num),'pic54.jpg')
        elseif fig_num == 55
            saveas(figure(fig_num),'pic55.jpg')
        elseif fig_num == 56
            saveas(figure(fig_num),'pic56.jpg')
        elseif fig_num == 57
            saveas(figure(fig_num),'pic57.jpg')
        elseif fig_num == 58
            saveas(figure(fig_num),'pic58.jpg')
        elseif fig_num == 59
            saveas(figure(fig_num),'pic59.jpg')
        elseif fig_num == 60
            saveas(figure(fig_num),'pic60.jpg')
        elseif fig_num == 61
            saveas(figure(fig_num),'pic61.jpg')
        elseif fig_num == 62
            saveas(figure(fig_num),'pic62.jpg')
        elseif fig_num == 63
            saveas(figure(fig_num),'pic63.jpg')
        elseif fig_num == 64
            saveas(figure(fig_num),'pic64.jpg')
        elseif fig_num == 65
            saveas(figure(fig_num),'pic65.jpg')
        elseif fig_num == 66
            saveas(figure(fig_num),'pic66.jpg')
        elseif fig_num == 67
            saveas(figure(fig_num),'pic67.jpg')
        elseif fig_num == 68
            saveas(figure(fig_num),'pic68.jpg')
        elseif fig_num == 69
            saveas(figure(fig_num),'pic69.jpg')
        elseif fig_num == 70
            saveas(figure(fig_num),'pic70.jpg')
        elseif fig_num == 71
            saveas(figure(fig_num),'pic71.jpg')
        elseif fig_num == 72
            saveas(figure(fig_num),'pic72.jpg')
        elseif fig_num == 73
            saveas(figure(fig_num),'pic73.jpg')
        elseif fig_num == 74
            saveas(figure(fig_num),'pic74.jpg')
        elseif fig_num == 75
            saveas(figure(fig_num),'pic75.jpg')
        elseif fig_num == 76
            saveas(figure(fig_num),'pic76.jpg')
        elseif fig_num == 77
            saveas(figure(fig_num),'pic77.jpg')
        elseif fig_num == 78
            saveas(figure(fig_num),'pic78.jpg')
                    elseif fig_num == 79
            saveas(figure(fig_num),'pic79.jpg')
        elseif fig_num == 80
            saveas(figure(fig_num),'pic80.jpg')
        elseif fig_num == 81
            saveas(figure(fig_num),'pic81.jpg')
        elseif fig_num == 82
            saveas(figure(fig_num),'pic82.jpg')
        elseif fig_num == 83
            saveas(figure(fig_num),'pic83.jpg')
        elseif fig_num == 84
            saveas(figure(fig_num),'pic84.jpg')
        elseif fig_num == 85
            saveas(figure(fig_num),'pic85.jpg')
        elseif fig_num == 86
            saveas(figure(fig_num),'pic86.jpg')
        elseif fig_num == 87
            saveas(figure(fig_num),'pic87.jpg')
        elseif fig_num == 88
            saveas(figure(fig_num),'pic88.jpg')
        elseif fig_num == 89
            saveas(figure(fig_num),'pic89.jpg')
        elseif fig_num == 90
            saveas(figure(fig_num),'pic90.jpg')
        elseif fig_num == 91
            saveas(figure(fig_num),'pic91.jpg')
        elseif fig_num == 92
            saveas(figure(fig_num),'pic92.jpg')
        elseif fig_num == 93
            saveas(figure(fig_num),'pic93.jpg')
        elseif fig_num == 94
            saveas(figure(fig_num),'pic94.jpg')
        elseif fig_num == 95
            saveas(figure(fig_num),'pic95.jpg')
        elseif fig_num == 96
            saveas(figure(fig_num),'pic96.jpg')
        elseif fig_num == 97
            saveas(figure(fig_num),'pic97.jpg')
        elseif fig_num == 98
            saveas(figure(fig_num),'pic98.jpg')
        elseif fig_num == 99
            saveas(figure(fig_num),'pic99.jpg')
        end
        
        
    end
    
    
    
end
