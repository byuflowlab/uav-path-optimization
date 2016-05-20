function color = color_b(color_var)

global green_fast summer cool copper parula_c;

color = zeros(length(color_var),1);

for i = 1 : length(color_var)
    
    
    if green_fast == 1
        color(i) = 0;
    elseif summer == 1
        color(i) = 0.4;
    elseif cool == 1
        color(i) = 1;
    elseif copper == 1
        color(i) = color_var(i)*0.5;
        elseif parula_c == 1
        [color(i), ~, ~] = parulacolor(color_var(i));
        
    else
        color(i) = 0;
    end
    
    if color(i) >= 1.0
        color(i) = 1;
    end
    
end


end