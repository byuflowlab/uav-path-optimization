function [e, g] = calc_f_opt_t(xi)

global xf; %final location of object
global num_path;
global x0;
global t;
global step_max; %note that this is the same value as max path length (delta time = 1 sec)
global step_min;

%x_last = real(xi(2*num_path,1)); y_last = real(xi(2*num_path,2));
x_last = xi(2*num_path,1); y_last = xi(2*num_path,2);

%calculate distance from final path segment's end point and final
%destination
D = ( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^0.5;

g = zeros(num_path*2,2);

g(num_path*2,1) = -(xf(1) - x_last)*( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^(-0.5);
g(num_path*2,2) = -(xf(1) - y_last)*( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^(-0.5);

% l_l = 0;
% 
% for i = 1 : num_path
% 
%     
%     if i == 1
%         p_prev = x0(1,:);
%         
%         for j = 1 : length(t)
%             %calculate position
%             p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
%             
%             %find distance from previous position to new position
%             %d = norm(p-p_prev);
%             d = ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^0.5;
%             
%             %add distance to total length
%             l_l = l_l + d;
%             
%             %change initial position
%             p_prev = p;
%         end
%         
%     else
%         p_prev = xi(2*i-2,:);
%         
%         for j = 1 : length(t)
%             %calculate position
%             p = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
%             
%             %find distance from previous position to new position
%             %d = norm(p-p_prev);
%             d = ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^0.5;
%             
%             %add distance to total length
%             l_l = l_l + d;
%             
%             %change initial position
%             p_prev = p;
%         end
%     end
%     
% end

e = D/step_max + (num_path); %where num_path is the time in seconds that the UAv is flying


end