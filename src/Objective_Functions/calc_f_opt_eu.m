function [c, g] = calc_f_opt_eu(xi)


global xf; %final location of object
global num_path;
global x0;
global t;
global initial;
global D_eta_opt;
global rho f W span eo;

x6 = real(xi(2*num_path,1)); y6 = real(xi(2*num_path,2));

%distance from final Bezier curve ending point to final destination
D = ( (xf(1) - x6)^2 + (xf(2) - y6)^2 )^0.5;

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
    
    L(k) = sum(l_l(:,k));
    
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
        v((length(t)-1)*(i-1)+j) = l_l(j,i)/(t(2)-t(1));
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

%find minimum d_l, and minimum efficiency
if initial == 1
    V_possible = 0.1 : 0.01 : 25;
    
    for i = 1 : length(V_possible)
        
        D_L = A*V_possible(i)^2 + B/V_possible(i)^2; % we want to maximize l_d, or minimize d_l
        
        eta_pos = calc_eff(V_possible(i));
        
        %calculate D_L/eta
        D_eta(i) = D_L/eta_pos;
    end
    
    %find optimal D_eta
    D_eta_opt = min(D_eta);
    
end
%----------------------------%

%calculate 'e' (defined in notes)
e = 0;
for i = 1 : length(v)
    e = e + (t(2)-t(1))*(v(i))*d_l(i)/eta(i);
end

for i = 1 : num_path
    for j = 1 : length(t)-1
    
        g = g + dl(:,:,(i-1)*(length(t)-1)+j)/eta(i)*(A*3*l_l(j,i)^2-B/l_l(j,i)^(-2));
        
    end
end

g(num_path*2,1) = g(num_path*2,1) -D_eta_opt*(xf(1) - x6)*( (xf(1) - x6)^2 + (xf(2) - y6)^2 )^(-0.5);
g(num_path*2,2) = g(num_path*2,2) -D_eta_opt*(xf(1) - y6)*( (xf(1) - x6)^2 + (xf(2) - y6)^2 )^(-0.5);

%calculate c
c = D*D_eta_opt + e;

end