function [c, ceq, gc, gceq] = calc_cons_acg(xi)

%global variables
global step_max;
global step_min;
global min_speed;
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
global l_l_last;

c = [];
ceq = [];
gc = [];
gceq = [];

ac = 5.0/(length(t));
if l_l_last == (min_speed)/(length(t))
    ac = 10.0/length(t);
end


%-----------------derivative constraints-------------%

for i = 1 : num_path
    
    if i == 1 % derivative equality constraint betweeen first segment and previous traversed segment
        P1 = xi(1,:);
        ceq(1) = 2*x0(1) - Pmid(1) - P1(1);
        ceq(2) = 2*x0(2) - Pmid(2) - P1(2);
        
        added_column1 = zeros(num_path*4,1);
        added_column2 = zeros(num_path*4,1);
        added_column1(1) = -1;
        added_column2(1+2*num_path) = -1;
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
        
        added_column2(2*i-3+2*num_path) = 2;
        added_column2(2*i-2+2*num_path) = -4;
        added_column2(2*i-1+2*num_path) = 2;
        
        gceq = [gceq added_column1 added_column2];
    end
    
end;


%--------------maximum/minimum step distance----------%
% step_max > length of line
% 0 > length of line - step_max;

for i = 1 : num_path
    
    l_l = zeros(length(t)-1,1);
    
    added_column = zeros(4*num_path,length(t)-1);
    
    if i == 1
        p_prev = x0(1,:);
        
        for j = 2 : length(t)
            %calculate position
            p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            
            %find distance from previous position to new position
            d = complex_norm(p,p_prev);
            
            %add distance to total length
            l_l(j-1) = d;
            
            
            added_column(1,j-1) = added_column(1,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            added_column(1+2*num_path,j-1) = added_column(1+2*num_path,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            added_column(2,j-1) = added_column(2,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
            added_column(2+2*num_path,j-1) = added_column(2+2*num_path,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            
            
            %change initial position
            p_prev = p;
            
            %acceleration constraints
            if j == 2
                c = [c -ac+l_l_last-l_l(j-1)];
                c = [c -ac-l_l_last+l_l(j-1)];
                
                %gradients of acceleration constraints
                gc = [gc -added_column(:,j-1)];
                gc = [gc added_column(:,j-1)];
                
            end
            
            if j > 2 && j < length(t)
                
                c = [c -ac+l_l(j-2)-l_l(j-1)];
                c = [c -ac-l_l(j-2)+l_l(j-1)];
                
                %gradients of acceleration constraints
                gc = [gc added_column(:,j-2)-added_column(:,j-1)];
                gc = [gc -added_column(:,j-2)+added_column(:,j-1)];
                
            end
            
        end
        
    else
        p_prev = xi(2*i-2,:);
        
        for j = 2 : length(t)
            %calculate position
            p = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
            
            %find distance from previous position to new position
            d = complex_norm(p,p_prev);
            
            %add distance to total length
            l_l(j-1) = d;
            
            added_column(2*i-2,j-1) = added_column(2*i-2,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*((1-t(j))^2-(1-t(j-1))^2);
            added_column(2*i-2+2*num_path,j-1) = added_column(2*i-2+2*num_path,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*((1-t(j))^2-(1-t(j-1))^2);
            added_column(2*i-1,j-1) = added_column(2*i-1,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            added_column(2*i-1+2*num_path,j-1) = added_column(2*i-1+2*num_path,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            added_column(2*i,j-1) = added_column(2*i,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
            added_column(2*i+2*num_path,j-1) = added_column(2*i+2*num_path,j-1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            
            %change initial position
            p_prev = p;
            
            %acceleration constraints
            
            if j > 2 && j < length(t)
                
                c = [c -ac+l_l(j-2)-l_l(j-1)];
                c = [c -ac-l_l(j-2)+l_l(j-1)];
                
                %gradients of acceleration constraints
                gc = [gc added_column(:,j-2)-added_column(:,j-1)];
                gc = [gc -added_column(:,j-2)+added_column(:,j-1)];
                
            end
        end
    end
    
    %add  min/ max constraints
    c = [c sum(l_l)-step_max step_min-sum(l_l)];
    
    added_column = added_column';
    
    gc = [gc sum(added_column)' -sum(added_column)'];
    
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
        
        %calculate gradient
        p = [];
        added_column = zeros(4*num_path,1);
        if k == 1
            p(1) = (1-t(j))^2*x0(1,1) + 2*(1-t(j))*t(j)*xi(1,1)+t(j)^2*xi(2,1);
            p(2) = (1-t(j))^2*x0(1,2) + 2*(1-t(j))*t(j)*xi(1,2)+t(j)^2*xi(2,2);
            added_column(1) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*2*(1-t(j))*t(j);
            added_column(1+2*num_path) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*2*(1-t(j))*t(j);
            added_column(2) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*t(j)^2;
            added_column(2+2*num_path) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*t(j)^2;
        else
            p(1) = (1-t(j))^2*xi(2*k-2,1) + 2*(1-t(j))*t(j)*xi(2*k-1,1)+t(j)^2*xi(2*k,1);
            p(2) = (1-t(j))^2*xi(2*k-2,2) + 2*(1-t(j))*t(j)*xi(2*k-1,2)+t(j)^2*xi(2*k,2);
            added_column(2*k-2) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*(1-t(j))^2;
            added_column(2*k-2+2*num_path) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*(1-t(j))^2;
            added_column(2*k-1) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*2*(1-t(j))*t(j);
            added_column(2*k-1+2*num_path) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*2*(1-t(j))*t(j);
            added_column(2*k) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(1) - obs_insight(i,1))*t(j)^2;
            added_column(2*k+2*num_path) = -0.5*( (p(1) - obs_insight(i,1))^2 + (p(2) - obs_insight(i,2))^2 )^(-0.5)*2*(p(2) - obs_insight(i,2))*t(j)^2;
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
                    
                    %p = [];
                    added_column = zeros(4*num_path,1);
                    
                    %p(1) = (1-t(i))^2*x0(1,1) + 2*(1-t(i))*t(i)*xi(1,1)+t(i)^2*xi(2,1);
                    %p(2) = (1-t(i))^2*x0(1,2) + 2*(1-t(i))*t(i)*xi(1,2)+t(i)^2*xi(2,2);
                    added_column(1) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(1) - d_o_cp(1,1))*2*(1-t(j))*t(j);
                    added_column(1+2*num_path) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(2) - d_o_cp(1,2))*2*(1-t(j))*t(j);
                    added_column(2) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(1) - d_o_cp(1,1))*t(j)^2;
                    added_column(2+2*num_path) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(2) - d_o_cp(1,2))*t(j)^2;
                    
                    gc = [gc added_column];
                    
                end
                
            else
                for i = 1 : length(t)
                    p = (1-t(i))^2*xi(2*k-2,:) + 2*(1-t(i))*t(i)*xi(2*k-1,:)+t(i)^2*xi(2*k,:);
                    
                    d_o_cp = obs_d_cp(j,:) + ((k-1) + t(i))*obs_d_v(j,:);
                    
                    d_min = complex_norm(p,d_o_cp);
                    
                    constraint = uav_ws+obs_d_s(j) - d_min;
                    
                    c = [c constraint];
                    
                    %p = [];
                    added_column = zeros(4*num_path,1);
                    
                    %p(1) = (1-t(i))^2*xi(2*k-2,1) + 2*(1-t(i))*t(i)*xi(2*k-1,1)+t(j)^2*xi(2*k,1);
                    %p(2) = (1-t(i))^2*xi(2*k-2,2) + 2*(1-t(i))*t(i)*xi(2*k-1,2)+t(j)^2*xi(2*k,2);
                    added_column(2*k-2) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(1) - d_o_cp(1,1))*(1-t(j))^2;
                    added_column(2*k-2+2*num_path) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(2) - d_o_cp(1,2))*(1-t(j))^2;
                    added_column(2*k-1) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(1) - d_o_cp(1,1))*2*(1-t(j))*t(j);
                    added_column(2*k-1+2*num_path) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(2) - d_o_cp(1,2))*2*(1-t(j))*t(j);
                    added_column(2*k) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(1) - d_o_cp(1,1))*t(j)^2;
                    added_column(2*k+2*num_path) = -0.5*( (p(1) - d_o_cp(1,1))^2 + (p(2) - d_o_cp(1,2))^2 )^(-0.5)*2*(p(2) - d_o_cp(1,2))*t(j)^2;
                    
                    gc = [gc added_column];
                    
                end
                
                
            end
            
        end
        
    end
    
end

% for k = 1 : num_path
%     
%     if k == 1
%         
%         for j = 1 : length(t)-2
%             
%             %calculate position
%             %                     p1 = x0(1,:); % t=0
%             %                     p2 = 0.25*x0(1,:) + 0.5*x_new(1,:,i)+0.25*x_new(2,:,i); %t = 0.5
%             %                     p3 = x_new(2,:,i); % t = 1.0
%             
%             p1 = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*x_new(1,:,i)+t(j)^2*x_new(2,:,i);
%             p2 = (1-t(j+1))^2*x0(1,:) + 2*(1-t(j+1))*t(j+1)*x_new(1,:,i)+t(j+1)^2*x_new(2,:,i);
%             p3 = (1-t(j+2))^2*x0(1,:) + 2*(1-t(j+2))*t(j+2)*x_new(1,:,i)+t(j+2)^2*x_new(2,:,i);
%             
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
%             
%             %calculate gc
%             
%             a = x1;
%             b = x2;
%             c = x3;
%             d = y1;
%             e = y2;
%             f = y3;
%             
%             dxc_da = (a^2*(e-f)-2*a*(b*(d-f)-c*(d-e))+b^2*(d-f)-(c^2+(d-f)*(e-f))*(d-e))*(e-f)/(2*(a*(e-f)-b*(d-f)+c*(d-e))^2);
%             dxc_db = (b^2*(d-f)-2*b*(a*(e-f)-c*(d-e))+a^2*(e-f)-(c^2+(d-f)*(e-f))*(d-e))*(e-f)/(2*(b*(d-f)-a*(e-f)+c*(d-e))^2);          
%             dxc_dc = (c^2*(d-e)-2*c*(a*(e-f)-b*(d-d))-a^2*(e-f)+(b^2+(d-e)*(e-f))*(d-f))*(d-e)/(2*(c*(d-e)+a*(e-f)+b*(d-f))^2);
%             
%             dxc_dd = -(d^2*(b-c)-2*d*(a*(e-f)+b*f-c*e)-a^2*(b-c)+a*(b^2-c^2+(e-f))-b^2*c+b*(c^2+f^2)-c*e^2)*(e-f)/(2*(d*(b-c)-a*(e-f)-b*f+c*e)^2);
%             dxc_de = -(e^2*(a-c)-2*e(a*f+b*(d-f)-c*d)+a^2*(b-c)-a*(b^2-c^2-f^2)+b^2*c-b*(c^2-(d+f)*(d-f))-c*d^2)*(d-f)/(2*(e*(a-c)-a*f-b*(d-f)+c*d)^2);
%             dxc_df = -(f^2*(a-b)-2*f*(a*e-b*d+c*(d-e))-a^2*(b-c)+a*(b^2-c^2+e^2)-b^2*c+b*(c^2-d^2)+c*(d+e)*(d-e))*(d-e)/(2*(f*(a-b)-a*e+b*d-c*(d-e))^2);
%             
%         end
%         
%     else
%         
%         for j = 1 : length(t)-2
%             
%             %calculate position
%             
%             
%             p1 = (1-t(j))^2*x_new(2*k-2,:,i) + 2*(1-t(j))*t(j)*x_new(2*k-1,:,i)+t(j)^2*x_new(2,:,i);
%             p2 = (1-t(j+1))^2*x_new(2*k-2,:,i) + 2*(1-t(j+1))*t(j+1)*x_new(2*k-1,:,i)+t(j+1)^2* x_new(2*k,:,i);
%             p3 = (1-t(j+2))^2*x_new(2*k-2,:,i) + 2*(1-t(j+2))*t(j+2)*x_new(2*k-1,:,i)+t(j+2)^2* x_new(2*k,:,i);
%             
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
%             
%         end
%         
%     end
%     
% end

end