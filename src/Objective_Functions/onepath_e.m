function [eu] = onepath_eu(x_guess_final)

global t;
global num_path;
global x0;

global xf; %final location of object
global initial;
global D_eta_opt;
global rho f W span eo;

line_length = 0;


p_prev = x_guess_final(1,:);
for i = 1 : num_path
    
    if i == 1
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*x_guess_final(1,:)+t(j)^2*x_guess_final(2,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            line_length = line_length + d;
            
            %change initial position
            p_prev = p;
            
            
        end
        
    else
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*x_guess_final(2*i-2,:) + 2*(1-t(j))*t(j)*x_guess_final(2*i-1,:)+t(j)^2*x_guess_final(2*i,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            line_length = line_length + d;
            
            %change initial position
            p_prev = p;
        end
        
    end
    
    L(i) = line_length;
end

%relate velocity to distance traveled
for i = 1 : num_path
    if i == 1
        v(i) = L(i); %*2;
    else
        v(i) = (L(i)-L(i-1)); %*2;  %this can be changed, depending on time step
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

%calculate 'e' (defined in notes)
e = 0;
for i = 1 : num_path
    e = e + (v(i))*d_l(i)/eta(i);
end


%calculate energy use
eu = e + D_eta_opt * norm (xf - x_guess_final(length(x_guess_final),:));

end