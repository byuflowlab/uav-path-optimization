function [c, ceq, gc, gceq] = calc_cons_acg(xi)

%global variables
global step_max;
global step_min;
global x0;
global t;
global obs;
global n_obs;
global obs_rad;
global Pmid;
global turn_r; %minimum turn radius
global num_path;
global Dynamic_Obstacles;
global uav_ws;

c = [];
ceq = [];
gc = [];
gceq = [];

%-----------------derivative constraints-------------%

for i = 1 : num_path
    
    if i == 1 % derivative equality constraint betweeen first segment and previous traversed segment
        P1 = xi(1,:);
        ceq(1) = 2*x0(1) - Pmid(1) - P1(1);
        ceq(2) = 2*x0(2) - Pmid(2) - P1(2);
        
        added_column1 = zeros(num_path*4,1);
        added_column2 = zeros(num_path*4,1);
        added_column1(1) = -1;
        added_column2(7) = -1;
        gceq = [gceq added_column1 added_column2];
        
    else
        %equality constraint of 1st derivative between two line sections
        
        
        P3 = xi(2*i-1,:);
        P2 = xi(2*i-2,:);
        P1 = xi(2*i-3,:);
        
        ceq(2*(i-1)+1) = 2*P3(1) - 4*P2(1) + 2*P1(1);
        ceq(2*(i-1)+2) = 2*P3(2) - 4*P2(2) + 2*P1(2);
        
        added_column1 = zeros(num_path*4,1);
        added_column2 = zeros(num_path*4,1);
        
        added_column1(2*i-3) = 2;
        added_column1(2*i-2) = -4;
        added_column1(2*i-1) = 2;
        
        added_column2(2*i+3) = 2;
        added_column2(2*i+4) = -4;
        added_column2(2*i+5) = 2;
        
        gceq = [gceq added_column1 added_column2];
    end
    
end;


%--------------maximum/minimum step distance----------%
% step_max > length of line
% 0 > length of line - step_max;

for i = 1 : num_path
    
    l_l = 0;
    
    added_column = zeros(4*num_path,1);
    
    if i == 1
        p_prev = x0(1,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            
            %find distance from previous position to new position
            d = complex_norm(p,p_prev);
            
            %add distance to total length
            l_l = l_l + d;
            
            if j > 1
                added_column(1) = added_column(1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
                added_column(7) = added_column(7) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
                added_column(2) = added_column(2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
                added_column(8) = added_column(8) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            end
            
            %change initial position
            p_prev = p;
        end
        
    else
        p_prev = xi(2*i-2,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
            
            %find distance from previous position to new position
            d = complex_norm(p,p_prev);
            
            %add distance to total length
            l_l = l_l + d;
            
            if j > 1
                added_column(2*i-2) = added_column(2*i-2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*((1-t(j))^2-(1-t(j-1))^2);
                added_column(2*i+4) = added_column(2*i+4) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*((1-t(j))^2-(1-t(j-1))^2);
                added_column(2*i-1) = added_column(2*i-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
                added_column(2*i+5) = added_column(2*i+5) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
                added_column(2*i) = added_column(2*i) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
                added_column(2*i+6) = added_column(2*i+6) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            end
            %change initial position
            p_prev = p;
        end
    end
    
    %add constraints
    c = [c l_l-step_max step_min - l_l];
    
    gc = [gc added_column -added_column];
    
end

%-------------static obstacle constraints-------------%



n_obs_insight = n_obs;
obs_insight = obs;
obs_r_insight = obs_rad;
removed = 0;
for i = 1 : n_obs
    if norm(x0-obs(i,:)) > step_max*num_path
        obs_insight(i-removed,:) = [];
        obs_r_insight(i-removed) = [];
        n_obs_insight = n_obs_insight - 1;
        removed = removed + 1;
    end
end


for k = 1 : num_path
    
    for i = 1 : n_obs_insight
        
        for j = 1 : length(t)
            
            if k == 1 % first path segment
                % location at point t on curve
                %p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,1:2);
                p(1) = (1-t(j))^2*x0(1,1) + 2*(1-t(j))*t(j)*xi(1,1)+t(j)^2*xi(2,1);
                p(2) = (1-t(j))^2*x0(1,2) + 2*(1-t(j))*t(j)*xi(1,2)+t(j)^2*xi(2,2);
            else % seuraaavat
                % location at point t on curve
                %p = (1-t(j))^2*xi(2*k-2,:) + 2*(1-t(j))*t(j)*xi(2*k-1,:)+t(j)^2*xi(2*k,:);
                p(1) = (1-t(j))^2*xi(2*k-2,1) + 2*(1-t(j))*t(j)*xi(2*k-1,1)+t(j)^2*xi(2*k,1);
                p(2) = (1-t(j))^2*xi(2*k-2,2) + 2*(1-t(j))*t(j)*xi(2*k-1,2)+t(j)^2*xi(2*k,2);
            end
            
            %distance from location at point t on curve to obstacle
            %d_p(j) = complex_norm(p,obs_insight(i,:));
            d_p(j) = ( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^0.5;
        end
        
        %choose minimum of all distances
        d_true = min(d_p);
        
        for j = 1 : length(t)
            if d_true == d_p(j)
                break
            end
        end
        
        %set distance as constraint
        % d_obs < d_true
        % 0 < d_true - d_obs
        c = [c uav_ws+obs_r_insight(i)-d_true];
        
        p = [];
        added_column = zeros(4*num_path,1);
        if k == 1
            p(1) = (1-t(j))^2*x0(1,1) + 2*(1-t(j))*t(j)*xi(1,1)+t(j)^2*xi(2,1);
            p(2) = (1-t(j))^2*x0(1,2) + 2*(1-t(j))*t(j)*xi(1,2)+t(j)^2*xi(2,2);
            added_column(1) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*2*(1-t(j))*t(j);
            added_column(7) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*2*(1-t(j))*t(j);
            added_column(2) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*t(j)^2;
            added_column(8) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*t(j)^2;
        else
            p(1) = (1-t(j))^2*xi(2*k-2,1) + 2*(1-t(j))*t(j)*xi(2*k-1,1)+t(j)^2*xi(2*k,1);
            p(2) = (1-t(j))^2*xi(2*k-2,2) + 2*(1-t(j))*t(j)*xi(2*k-1,2)+t(j)^2*xi(2*k,2);
            added_column(2*k-2) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*(1-t(j))^2;
            added_column(2*k+4) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*(1-t(j))^2;
            added_column(2*k-1) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*2*(1-t(j))*t(j);
            added_column(2*k+5) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*2*(1-t(j))*t(j);
            added_column(2*k) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*t(j)^2;
            added_column(2*k+6) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*t(j)^2;
        end
        gc = [gc added_column];
        
    end
    
    
end

%----------dynamic obstacle constraints------------%
% This steps on each point of the first path, finds the smallest distance
% between each point and the path traversed by the dynamic obstacle, and
% makes sure that it doesn't collide with the object.
if Dynamic_Obstacles == 1
    
    global n_obsd obs_d_v obs_d_s obs_d_cp;
    
    for j = 1 : n_obsd
        
        for k = 1 : num_path
            
            if k == 1
                for i = 1 : length(t)
                    
                    p = (1-t(i))^2*x0(1,:) + 2*(1-t(i))*t(i)*xi(1,:)+t(i)^2*xi(2,:);
                    
                    d_o_cp = obs_d_cp(j,:) + t(i)*obs_d_v(j,:);
                    
                    d_min = complex_norm(p,d_o_cp);
                    
                    constraint = uav_ws+obs_d_s(j) - d_min;
                    
                    c = [c constraint];
                    
                end
                
            else
                for i = 1 : length(t)
                    p = (1-t(i))^2*xi(2*k-2,:) + 2*(1-t(i))*t(i)*xi(2*k-1,:)+t(i)^2*xi(2*k,:);
                    
                    d_o_cp = obs_d_cp(j,:) + ((k-1) + t(i))*obs_d_v(j,:);
                    
                    d_min = complex_norm(p,d_o_cp);
                    
                    constraint = uav_ws+obs_d_s(j) - d_min;
                    
                    c = [c constraint];
                    
                    
                end
            end
        end
        
    end
    
end
%--------constraints for turn radius---------%

% for i = 1 : num_path
%
%     if i == 1
%
%         for j = 1 : length(t)-2
%             %calculate position
%             p1 = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
%             p2 = (1-t(j+1))^2*x0(1,:) + 2*(1-t(j+1))*t(j+1)*xi(1,:)+t(j+1)^2*xi(2,:);
%             p3 = (1-t(j+2))^2*x0(1,:) + 2*(1-t(j+2))*t(j+2)*xi(1,:)+t(j+2)^2*xi(2,:);
%             x1 = p1(1);
%             y1 = p1(2);
%             x2 = p2(1);
%             y2 = p2(2);
%             x3 = p3(1);
%             y3 = p3(2);
%
%             % check http://www.intmath.com/applications-differentiation/8-radius-curvature.php
%             %x1 = 1; y1 = 1; x2 = 2; y2 = 3; x3 = 3; y3 = 8;
%
%             m1 = (y2-y1)/(x2-x1);
%             m2 = (y3-y2)/(x3-x2);
%
%             xc = (m1*m2*(y1-y3)+m2*(x1+x2)-m1*(x2+x3))/(2*(m2-m1));
%             yc = -1/m1*(xc-(x1+x2)/2)+(y1+y2)/2;
%
%             r = ((x2-xc)^2+(y2-yc)^2)^0.5;
%
%
%             if abs(m1-m2) < 0.001
%
%                 r = 1000000;
%
%             end
%
%             c = [c turn_r-r];
%         end
%
%     else
%
%         for j = 1 : length(t)-2
%             %calculate position
%             p1 = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
%             p2 = (1-t(j+1))^2*xi(2*i-2,:) + 2*(1-t(j+1))*t(j+1)*xi(2*i-1,:)+t(j+1)^2*xi(2*i,:);
%             p3 = (1-t(j+2))^2*xi(2*i-2,:) + 2*(1-t(j+2))*t(j+2)*xi(2*i-1,:)+t(j+2)^2*xi(2*i,:);
%             x1 = p1(1);
%             y1 = p1(2);
%             x2 = p2(1);
%             y2 = p2(2);
%             x3 = p3(1);
%             y3 = p3(2);
%
%             % check http://www.intmath.com/applications-differentiation/8-radius-curvature.php
%             %x1 = 1; y1 = 1; x2 = 2; y2 = 3; x3 = 3; y3 = 8;
%
%             m1 = (y2-y1)/(x2-x1);
%             m2 = (y3-y2)/(x3-x2);
%
%             xc = (m1*m2*(y1-y3)+m2*(x1+x2)-m1*(x2+x3))/(2*(m2-m1));
%             yc = -1/m1*(xc-(x1+x2)/2)+(y1+y2)/2;
%
%             r = ((x2-xc)^2+(y2-yc)^2)^0.5;
%
%             if abs(m1-m2) < 0.001
%
%                 r = 1000000;
%
%             end
%
%             c = [c turn_r-r];
%
%         end
%     end
%
% end

end