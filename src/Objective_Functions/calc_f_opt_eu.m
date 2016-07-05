function [c, g] = calc_f_opt_eu(xi)


global xf; %final location of object
global num_path;
global x0;
global t;
global initial;
global D_eta_opt;
global rho f W span eo;

dt = t(2) - t(1);

x_last = xi(2*num_path,1);
y_last = xi(2*num_path,2);

%distance from final Bezier curve ending point to final destination
D = ( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^0.5;

g = zeros(num_path*2,2);

%fuel efficiency

%calculate step distance / velocity of each segment being planned

l_l = zeros(length(t)-1,num_path);
dl = zeros(num_path*2,2,num_path*(length(t)-1));

p_prev = x0(1,:);

for k = 1 : num_path
    
    if k == 1
        
        for j = 2 : length(t)
            %calculate position
            p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            
            %find distance from previous position to new position
            %d = norm(p-p_prev);
            d = ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^0.5;
            
            %add distance to total length
            l_l(j-1,k) = d;
            
            dl(1,1,(k-1)*(length(t)-1)+(j-1)) = dl(1,1,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            dl(1,2,(k-1)*(length(t)-1)+(j-1)) = dl(1,2,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            dl(2,1,(k-1)*(length(t)-1)+(j-1)) = dl(2,1,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
            dl(2,2,(k-1)*(length(t)-1)+(j-1)) = dl(2,2,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            
            %change initial position
            p_prev = p;
            
        end
        
    else
        
        for j = 2 : length(t)
            
            %calculate position
            p = (1-t(j))^2*xi(2*k-2,:) + 2*(1-t(j))*t(j)*xi(2*k-1,:)+t(j)^2*xi(2*k,:);
            
            %find distance from previous position to new position
            %d = norm(p-p_prev);
            d = ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^0.5;
            
            %add distance to total length
            l_l(j-1,k) = d;
            
            dl(2*k-2,1,(k-1)*(length(t)-1)+(j-1)) = dl(2*k-2,1,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*((1-t(j))^2-(1-t(j-1))^2);
            dl(2*k-2,2,(k-1)*(length(t)-1)+(j-1)) = dl(2*k-2,2,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*((1-t(j))^2-(1-t(j-1))^2);
            dl(2*k-1,1,(k-1)*(length(t)-1)+(j-1)) = dl(2*k-1,1,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            dl(2*k-1,2,(k-1)*(length(t)-1)+(j-1)) = dl(2*k-1,2,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(2*(1-t(j))*t(j)-2*(1-t(j-1))*t(j-1));
            dl(2*k,1,(k-1)*(length(t)-1)+(j-1)) = dl(2*k,1,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(1)-p_prev(1))*(t(j)^2-t(j-1)^2);
            dl(2*k,2,(k-1)*(length(t)-1)+(j-1)) = dl(2*k,2,(k-1)*(length(t)-1)+(j-1)) + ((p(1)-p_prev(1))^2+(p(2)-p_prev(2))^2)^(-0.5)*(p(2)-p_prev(2))*(t(j)^2-t(j-1)^2);
            
            %change initial position
            p_prev = p;
            
        end
        
    end
    
    %L(k) = sum(l_l(:,k));
    
end

%relate velocity to distance traveled
% for i = 1 : num_path
%     if i == 1
%         v(i) = L(i); %*2;
%     else
%         v(i) = (L(i)-L(i-1)); %*2;  %this can be changed, depending on time step
%     end
% end

v = zeros((length(t)-1)*num_path,1);

for i = 1 : num_path
   for j = 1 : length(t)-1
        v((length(t)-1)*(i-1)+j) = l_l(j,i)/dt;
   end
end

%Defined in paper (2nd column, page 2)
A = rho*f/(2*W);
B = 2*W/(rho*span^2*pi*eo);

%calculate l_d
for i = 1 : length(v)
    
    d_l(i) = A*v(i)^2 + B/v(i)^2; % we want to maximize l_d, or minimize d_l
    
    l_d(i) = d_l(i)^(-1);
    
end

%calculate overall efficiency
for i = 1 : length(v)
    
    eta(i) = calc_eff(v(i));
    
end

%----------------------------%

%calculate 'e' (defined in notes)
e = 0;
a = -0.0024;
b = 0.084;
c = -0.9;
d = 3.6;
for i = 1 : length(v)
    e = e + dt*(v(i))*d_l(i)/eta(i); %A*v(i)^3*dt/eta(i);
end

for i = 1 : num_path
    for j = 1 : length(t)-1
        
        top1 = A*l_l(j,i)^3/dt^2;
        dtop1 = A*3*l_l(j,i)^2*dl(:,:,(i-1)*(length(t)-1)+j)/(dt^2);
        bot = eta((i-1)*(length(t)-1)+j);
        
        if v((i-1)*(length(t)-1)+j) < 10
            dbot = 0.06/dt*dl(:,:,(i-1)*(length(t)-1)+j);
        elseif v((i-1)*(length(t)-1)+j) > 20
            dbot = 0;
        else
        dbot = (3*(-0.0024)*l_l(j,i)^2/dt^3 + 2*0.084/dt^2*l_l(j,i) - 0.9/dt)*dl(:,:,(i-1)*(length(t)-1)+j);
        end
        
        top2 = B*dt^2*l_l(j,i)^(-1);
        dtop2 = -B*dt^2*l_l(j,i)^(-2)*dl(:,:,(i-1)*(length(t)-1)+j);
        
        g = g + (bot*dtop1 - top1*dbot)/bot^2 + (bot*dtop2 - top2*dbot)/bot^2; %dl(:,:,(i-1)*(length(t)-1)+j)/eta((i-1)*(length(t)-1)+j)*(A*3*l_l(j,i)^2/dt^2-B/l_l(j,i)^(-2)*dt^2);
    end
end

g(num_path*2,1) = g(num_path*2,1) - D_eta_opt*(xf(1) - x_last)*( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^(-0.5);
g(num_path*2,2) = g(num_path*2,2) - D_eta_opt*(xf(1) - y_last)*( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^(-0.5);

%calculate c
c = D*D_eta_opt + e;

end