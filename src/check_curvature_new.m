function c = check_curvature(i)

%-------global variables----------%
global x0; %current starting point, Path_bez
global t; %parameterization variable
global turn_r; %minimum turn radius
global num_path; %number of segments optimized
global x_new;


%--------constraints for turn radius---------%
c = [];
for k = 1 : num_path
    
    if k == 1
        
        for j = 1 : length(t)-2
            
            %calculate position
            %                     p1 = x0(1,:); % t=0
            %                     p2 = 0.25*x0(1,:) + 0.5*x_new(1,:,i)+0.25*x_new(2,:,i); %t = 0.5
            %                     p3 = x_new(2,:,i); % t = 1.0
            
            p1 = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*x_new(1,:,i)+t(j)^2*x_new(2,:,i);
            p2 = (1-t(j+1))^2*x0(1,:) + 2*(1-t(j+1))*t(j+1)*x_new(1,:,i)+t(j+1)^2*x_new(2,:,i);
            p3 = (1-t(j+2))^2*x0(1,:) + 2*(1-t(j+2))*t(j+2)*x_new(1,:,i)+t(j+2)^2*x_new(2,:,i);
            
            x1 = p1(1);
            y1 = p1(2);
            x2 = p2(1);
            y2 = p2(2);
            x3 = p3(1);
            y3 = p3(2);
            
            % check http://www.intmath.com/applications-differentiation/8-radius-curvature.php
            %x1 = 1; y1 = 1; x2 = 2; y2 = 3; x3 = 3; y3 = 8;
            
            m1 = (y2-y1)/(x2-x1);
            m2 = (y3-y2)/(x3-x2);
            
            xc = (m1*m2*(y1-y3)+m2*(x1+x2)-m1*(x2+x3))/(2*(m2-m1));
            yc = -1/m1*(xc-(x1+x2)/2)+(y1+y2)/2;
            
            r = ((x2-xc)^2+(y2-yc)^2)^0.5;
            
            
            if abs(m1-m2) < 0.001
                
                r = 1000000;
                
            end
            
            c = [c turn_r-r];
            
        end
        
    else
        
        for j = 1 : length(t)-2
            
            %calculate position
            
            
            p1 = (1-t(j))^2*x_new(2*k-2,:,i) + 2*(1-t(j))*t(j)*x_new(2*k-1,:,i)+t(j)^2*x_new(2,:,i);
            p2 = (1-t(j+1))^2*x_new(2*k-2,:,i) + 2*(1-t(j+1))*t(j+1)*x_new(2*k-1,:,i)+t(j+1)^2* x_new(2*k,:,i);
            p3 = (1-t(j+2))^2*x_new(2*k-2,:,i) + 2*(1-t(j+2))*t(j+2)*x_new(2*k-1,:,i)+t(j+2)^2* x_new(2*k,:,i);
            
            x1 = p1(1);
            y1 = p1(2);
            x2 = p2(1);
            y2 = p2(2);
            x3 = p3(1);
            y3 = p3(2);
            
            % check http://www.intmath.com/applications-differentiation/8-radius-curvature.php
            %x1 = 1; y1 = 1; x2 = 2; y2 = 3; x3 = 3; y3 = 8;
            
            m1 = (y2-y1)/(x2-x1);
            m2 = (y3-y2)/(x3-x2);
            
            xc = (m1*m2*(y1-y3)+m2*(x1+x2)-m1*(x2+x3))/(2*(m2-m1));
            yc = -1/m1*(xc-(x1+x2)/2)+(y1+y2)/2;
            
            r = ((x2-xc)^2+(y2-yc)^2)^0.5;
            
            if abs(m1-m2) < 0.001
                
                r = 1000000;
                
            end
            
            c = [c turn_r-r];
            
            
        end
        
    end
    
end

end