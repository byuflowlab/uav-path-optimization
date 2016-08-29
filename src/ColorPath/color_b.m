function color = color_b(color_var)

global green_fast;
global summer_c cool_c copper_c parula_c winter_c;

color = zeros(length(color_var),1);

for i = 1 : length(color_var)
    
    
    if green_fast == 1
        color(i) = 0;
    elseif summer_c == 1
        color(i) = 0.4;
    elseif cool_c == 1
        color(i) = 1;
    elseif copper_c == 1
        color(i) = color_var(i)*0.5;
    elseif winter_c == 1
        color(i) = 1-0.5*color_var(i);
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