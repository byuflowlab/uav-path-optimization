function [c] = opt_e(xi)


global xf; %final location of object
global num_path;
global x0;
global t;
global initial;
global D_eta_opt;

x6 = real(xi(2*num_path,1)); y6 = real(xi(2*num_path,2));

%distance from final Bezier curve ending point to final destination
D = ( (xf(1) - x6)^2 + (xf(2) - y6)^2 )^0.5;

%fuel efficiency

%calculate step distance / velocity of each segment being planned

l_l = 0;
p_prev = x0(1,:);

for k = 1 : num_path
    
    if k == 1
        
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
        
        for j = 1 : length(t)
            
            %calculate position
            p = (1-t(j))^2*xi(2*k-2,:) + 2*(1-t(j))*t(j)*xi(2*k-1,:)+t(j)^2*xi(2*k,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            l_l = l_l + d;
            
            %change initial position
            p_prev = p;
            
        end
        
    end
    
    L(k) = l_l;
    
end

%relate velocity to distance traveled
for i = 1 : num_path
    if i == 1
        v(i) = L(i)*2;
    else
        v(i) = (L(i)-L(i-1))*2;  %this can be changed, depending on time step
    end
end

%parameter values
rho = 1.225; %air density
f = .2;   %equivalent parasite area
W = 10; %weight of aircraft
b = .20;   %span
eo = 0.9; %Oswald's efficiency factor


%Defined in paper (2nd column, page 2)
A = rho*f/(2*W);
B = 2*W/(rho*b^2*pi*eo);

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
for i = 1 : num_path
    e = e + (v(i)/2)*d_l(i)/eta(i);
end


%calculate c
c = D*D_eta_opt + e;

end