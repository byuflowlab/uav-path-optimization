function [f, g] = calc_f_opt_d(xi)

global xf; %final location of object
global num_path;
global x0;
global t;

g = zeros(num_path*2,2);

% x_last = real(xi(2*num_path,1));
% y_last = real(xi(2*num_path,2));
x_last = xi(2*num_path,1);
y_last = xi(2*num_path,2);

%calculate distance from final path segment's end point and final
%destination
D = ( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^0.5;

g(num_path*2,1) = -(xf(1) - x_last)*( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^(-0.5);
g(num_path*2,2) = -(xf(1) - y_last)*( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^(-0.5);

%calculate distance traveled by UAV
%l_l = zeros(num_path,1);
l_l = 0;

for i = 1 : num_path
    
    
    if i == 1
        p_prev = x0(1,:);
        
        for j = 2 : length(t)
            %calculate position
            %p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            p(1) = (1-t(j))^2*x0(1,1) + 2*(1-t(j))*t(j)*xi(1,1)+t(j)^2*xi(2,1);
            p(2) = (1-t(j))^2*x0(1,2) + 2*(1-t(j))*t(j)*xi(1,2)+t(j)^2*xi(2,2);
            
            %find distance from previous position to new position
            %d = norm(p-p_prev);
            d = ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^0.5;
            
            %add distance to total length
            l_l = l_l + d;
            
            
            g(1,1) = g(1,1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            g(1,2) = g(1,2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            g(2,1) = g(2,1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
            g(2,2) = g(2,2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            
            
            %change initial position
            p_prev = p;
            
            
        end
        
    else
        p_prev = xi(2*i-2,:);
        
        for j = 2 : length(t)
            %calculate position
            %p = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
            p(1) = (1-t(j))^2*xi(2*i-2,1) + 2*(1-t(j))*t(j)*xi(2*i-1,1)+t(j)^2*xi(2*i,1);
            p(2) = (1-t(j))^2*xi(2*i-2,2) + 2*(1-t(j))*t(j)*xi(2*i-1,2)+t(j)^2*xi(2*i,2);
            
            %find distance from previous position to new position
            %d = norm(p-p_prev);
            d = ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^0.5;
            
            %add distance to total length
            l_l = l_l + d;
            
            
            g(2*i-2,1) = g(2*i-2,1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*((1-t(j))^2-(1-t(j-1))^2);
            g(2*i-2,2) = g(2*i-2,2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*((1-t(j))^2-(1-t(j-1))^2);
            g(2*i-1,1) = g(2*i-1,1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            g(2*i-1,2) = g(2*i-1,2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            g(2*i,1) = g(2*i,1) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
            g(2*i,2) = g(2*i,2) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            
            
            %change initial position
            p_prev = p;
        end
    end
    
end

% sum of distance to final destination (D) and length of planned path (l_l)
f = D + l_l;


end