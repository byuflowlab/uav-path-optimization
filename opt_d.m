function [e] = opt_d(xi)

%alpha (penalty term for curviness)
alpha = 10;

global xf; %final location of object
global num_path;
global x0;
global t;

x_last = real(xi(2*num_path,1)); y_last = real(xi(2*num_path,2));

%calculate distance from final path segment's end point and final
%destination
D = ( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^0.5;

%calculate distance traveled by UAV 
%l_l = zeros(num_path,1);
l_l = 0;

for i = 1 : num_path

    
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
    
end

La = l_l;
% for i = 1 : length(l_l);
% 
%     La = La + l_l(i);
%     
% end

%calculate distance from starting point of first segment to ending point of
%final segment
Ls = norm(xi(2*num_path,:)-x0);

%calculate total cost, e
%e = D + alpha*(La-Ls);

%new distance function is sum of distance to final destination (D) and
%length of planned path (l_l)
e = D + l_l;


%gradients
% g = zeros(2*num_path,2);
% g(2*num_path,1) = (xf(1)-x_last)*(x_last^2 + 2*x_last*xf(1)+xf(1)^2+(xf(2)-y_last)^2)^(-0.5);
% g(2*num_path,2) = (xf(2)-y_last)*(y_last^2 + 2*y_last*xf(2)+xf(2)^2+(xf(1)-x_last)^2)^(-0.5);


end