function [c, ceq] = final_con(xi)

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
global xf;
global Dynamic_Obstacles;
global uav_ws;
global lr;
c = [];

%--------------maximum/minimum step distance----------%
% step_max > length of line
% 0 > length of line - step_max;

for i = 1 : num_path
    
    l_l = 0;
    
    if i == 1
        p_prev = x0(1,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            l_l = l_l + d;
            
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
            l_l = l_l + d;
            
            %change initial position
            p_prev = p;
        end
    end
    
    %add constraints
    c = [c l_l-step_max step_min*0.75 - l_l];
    
end

%-------------static obstacle constraints-------------%

obs_f = 0; %to give a little distance between path line and obstacles

for k = 1 : num_path
    
    for i = 1 : n_obs
        
        for j = 1 : length(t)
            
            if k == 1 % first path segment
                % location at point t on curve
                p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,1:2);
                
            else % seuraaavat
                % location at point t on curve
                p = (1-t(j))^2*xi(2*k-2,:) + 2*(1-t(j))*t(j)*xi(2*k-1,:)+t(j)^2*xi(2*k,:);
                
            end
            
            %distance from location at point t on curve to obstacle
            d_p(j) = norm(p-obs(i,:));
        end
        
        %choose minimum of all distances
        d_true = min(d_p);
        
        %set distance as constraint
        % d_obs < d_true
        % 0 < d_true - d_obs
        c = [c uav_ws+obs_rad(i)+obs_f-d_true];
    end
    
    
end

%-----------------derivative constraints-------------%

for i = 1 : num_path
    
    if i == 1 % derivative equality constraint betweeen first segment and previous traversed segment
        P1 = xi(1,:);
        ceq(1) = 2*x0(1) - Pmid(1) - P1(1);
        ceq(2) = 2*x0(2) - Pmid(2) - P1(2);
    else
        %equality constraint of 1st derivative between two line sections
        
        
        P3 = xi(2*i-1,:);
        P2 = xi(2*i-2,:);
        P1 = xi(2*i-3,:);
        
        ceq(2*i+1) = 2*P3(1) - 4*P2(1) + 2*P1(1);
        ceq(2*i+2) = 2*P3(2) - 4*P2(2) + 2*P1(2);
    end
    
end;

%final value is equal to xf
% equality_constraint1 = xf(1) - xi(2*num_path,1);
% equality_constraint2 = xf(2) - xi(2*num_path,2);
% 
% ceq = [ceq equality_constraint1 equality_constraint2];

c = [c norm(xf-xi(2*num_path,:))-lr];

%-------------static obstacle constraints-------------%

obs_f = 0.1; %to give a little distance between path line and obstacles

for k = 1 : num_path
    
    for i = 1 : n_obs
        
        for j = 1 : length(t)
            
            if k == 1 % first path segment
                % location at point t on curve
                p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,1:2);
                
            else % seuraaavat
                % location at point t on curve
                p = (1-t(j))^2*xi(2*k-2,:) + 2*(1-t(j))*t(j)*xi(2*k-1,:)+t(j)^2*xi(2*k,:);
                
            end
            
            %distance from location at point t on curve to obstacle
            d_p(j) = norm(p-obs(i,:));
        end
        
        %choose minimum of all distances
        d_true = min(d_p);
        
        %set distance as constraint
        % d_obs < d_true
        % 0 < d_true - d_obs
        c = [c uav_ws+obs_rad(i)+obs_f-d_true];
    end
    
    
end

%----------dynamic obstacle constraints------------%
% This steps on each point of the first path, finds the smallest distance
% between each point and the path traversed by the dynamic obstacle, and
% makes sure that it doesn't collide with the object.
if Dynamic_Obstacles == 1
    
    global n_obsd obs_d_v obs_d_s obs_d_cp;
    for k = 1 : num_path
        
        if k == 1
            for i = 1 : length(t)
                p = (1-t(i))^2*x0(1,:) + 2*(1-t(i))*t(i)*xi(1,:)+t(i)^2*xi(2,1:2);
                
                for j = 1 : n_obsd
                    
                    d_o_cp = obs_d_cp(j,:) + t(i)*obs_d_v(j,:);
                    
                    d_min = norm(p-d_o_cp);
                    
                    constraint = uav_ws+obs_d_s(j) - d_min;
                    
                    c = [c constraint];
                    
                end
            end
            
        else
            for i = 1 : length(t)
                p = (1-t(i))^2*xi(2*k-2,:) + 2*(1-t(i))*t(i)*xi(2*k-1,:)+t(i)^2*xi(2*k,1:2);
                
                for j = 1 : n_obsd
                    
                    d_o_cp = obs_d_cp(j,:) + ((k-0) + t(i))*obs_d_v(j,:);
                    
                    d_min = norm(p-d_o_cp);
                    
                    constraint = uav_ws+obs_d_s(j) - d_min;
                    
                    c = [c constraint];
                    
                end
            end
        end
    end
    
end

%--------constraints for turn radius---------%
%tata pitaa tarkistaa, mutta nyt se toimii hyvin
for i = 1 : num_path
    
    if i == 1
        
        t_d = norm(x0 - xi(2,:));
        
        d_1 = norm(x0 - xi(1,:));
        
        d_2 = norm(xi(2,:) - xi(1,:));
        
    else
        
        t_d = norm(xi(2*i,:) - xi(2*i-2,:));
        
        d_1 = norm(xi(2*i-2,:) - xi(2*i-1,:));
        
        d_2 = norm(xi(2*i,:) - xi(2*i-1,:));
        
    end
    
    constraint = 0.5*step_max - t_d;
    constraint1 = 0.25*step_max - d_1;
    constraint2 = 0.25*step_max - d_2;
    
    c = [c constraint constraint1 constraint2];
end


end