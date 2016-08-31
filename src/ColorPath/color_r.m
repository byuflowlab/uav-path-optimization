function color = color_r(color_var)

global green_fast;
global summer_c cool_c copper_c parula_c winter_c;

color = zeros(length(color_var),1);

for i = 1 : length(color_var)
    
    
    if summer_c == 1
        color(i) = color_var(i);
    elseif cool_c == 1
        color(i) = color_var(i);
    elseif copper_c == 1
        color(i) = color_var(i)*(65/50);
    elseif parula_c == 1
        [~, ~, color(i)] = parulacolor(color_var(i));
    elseif winter_c == 1
        color(i) = 0;
    else
        color(i) = 1-color_var(i);
    end
    
    if color(i) >= 1.0
        color(i) = 1;
    end
    
end


end