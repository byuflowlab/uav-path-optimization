function color = color_g(color_var)

global green_fast;
global summer_c cool_c copper_c parula_c winter_c blue_red blue_magenta_red green_purple blue_gray_red;
global shortened_parula_c shortened_viridis_c shortened_inferno_c;

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
            elseif shortened_parula_c == 1
        [~, color(i), ~] = shortened_parulacolor(color_var(i));
            elseif shortened_viridis_c == 1
        [~, color(i), ~] = shortened_viridiscolor(color_var(i));
                    elseif shortened_inferno_c == 1
        [~, color(i), ~] = shortened_infernocolor(color_var(i));
    elseif blue_red == 1
        color(i) = 0;
    elseif blue_magenta_red == 1
        color(i) = 0;
        elseif green_purple == 1
        color(i) = 1-color_var(i);    
       elseif blue_gray_red == 1
        color(i) = 1-color_var(i);
    end
    
    if color(i) >= 1.0
        color(i) = 1;
    end
    
    if color(i) <= 0.0
        color(i) = 0;
    end
    
end


end