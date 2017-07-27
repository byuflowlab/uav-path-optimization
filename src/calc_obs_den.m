function obs_density = calc_obs_den(n_obs, obs, obs_rad, uav_ws)

np = 500;

x = linspace(0,100,np);
y = linspace(0,100,np);

field = zeros(np);

for i = 1 : length(x)
for j = 1 : length(y)
    
    for k = 1 : n_obs
    
        if ((obs(k,1)-x(i))^2+(obs(k,2)-y(i))^2)^0.5 < (obs_rad(k))
            field(i,j) = 1;
        end
        
    end
    
end  
end

obs_density = sum(sum(field))/np^2;

end
