function color = color_g(color_var)

global green_fast;
global summer_c cool_c copper_c parula_c winter_c blue_red blue_magenta_red;

color = zeros(length(color_var),1);

for i = 1 : length(color_var)
    

    if summer_c == 1
        color(i) = color_var(i)*0.5 + 0.5;
    elseif cool_c == 1
        color(i) = 1-color_var(i);
    elseif copper_c == 1
        color(i) = color_var(i)*0.8;
    elseif winter_c == 1
        color(i) = color_var(i);
    elseif parula_c == 1
        [~,color(i), ~] = parulacolor(color_var(i));   
    elseif blue_red == 1
        color(i) = 0;
    elseif blue_magenta_red == 1
        color(i) = 0;
    end
    
    if color(i) >= 1.0
        color(i) = 1;
    end
    
    if color(i) <= 0.0
        color(i) = 0;
    end
    
end


end