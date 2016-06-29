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
%             %calculate matrices - http://mathworld.wolfram.com/Circle.html
%             a = [x1, y1, 1; x2, y2, 1; x3, y3, 1];
%             d = -[x1^2+y1^2, y1, 1; x2^2+y2^2, y2, 1; x3^2+y3^2, y3, 1];
%             e = -[x1^2+y1^2, x1, 1; x2^2+y2^2, x2, 1; x3^2+y3^2, x3, 1];
%             f = -[x1^2+y1^2, x1, y1; x2^2+y2^2, x2, y2; x3^2+y3^2, x3, y3];
%
%             r = ((d^2+e^2)/(4*a^2) - f/a)^0.5;
%             r = r(1,1);
%             if isnan(r)
%                 r = 1*10^9;
%             end
%
%             c = [c turn_r-abs(r)];
%
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
%             %calculate matrices - http://mathworld.wolfram.com/Circle.html
%             a = [x1, y1, 1; x2, y2, 1; x3, y3, 1];
%             d = -[x1^2+y1^2, y1, 1; x2^2+y2^2, y2, 1; x3^2+y3^2, y3, 1];
%             e = -[x1^2+y1^2, x1, 1; x2^2+y2^2, x2, 1; x3^2+y3^2, x3, 1];
%             f = -[x1^2+y1^2, x1, y1; x2^2+y2^2, x2, y2; x3^2+y3^2, x3, y3];
%
%             r = ((d^2+e^2)/(4*a^2) - f/a)^0.5;
%             r = r(1,1);
%             if isnan(r)
%                 r = 1*10^9;
%             end
%
%             c = [c turn_r-abs(r)];
%         end
%     end
%
% end

%tata pitaa tarkistaa, mutta nyt se toimii hyvin
% for i = 1 : num_path
% 
%     if i == 1
% 
%         t_d = complex_norm(x0 , xi(2,:));
% 
%         d_1 = complex_norm(x0 , xi(1,:));
% 
%         d_2 = complex_norm(xi(2,:) , xi(1,:));
% 
%     else
% 
%         t_d = complex_norm(xi(2*i,:) , xi(2*i-2,:));
% 
%         d_1 = complex_norm(xi(2*i-2,:) , xi(2*i-1,:));
% 
%         d_2 = complex_norm(xi(2*i,:) , xi(2*i-1,:));
% 
%     end
% 
%     constraint = 0.5*step_max - t_d;
%     constraint1 = 0.25*step_max - d_1;
%     constraint2 = 0.25*step_max - d_2;
% 
%     c = [c constraint constraint1 constraint2];
% end
