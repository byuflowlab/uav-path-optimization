function color = color_g(color_var)

global green_fast summer cool copper parula_c;

color = zeros(length(color_var),1);

for i = 1 : length(color_var)
    
    
    if green_fast == 1
        color(i) = color_var(i)*2;
        
    elseif summer == 1
        color(i) = color_var(i)*0.5 + 0.5;
    elseif cool == 1
        color(i) = 1-color_var(i);
    elseif copper == 1
        color(i) = color_var(i)*0.8;
        elseif parula_c == 1
        [~,color(i), ~] = parulacolor(color_var(i));
        
    else
        color(i) = (1-color_var(i))*2;
    end
    
    if color(i) >= 1.0
        color(i) = 1;
    end
    
end


end