function [total_distance] = evaluate_solution(Path_bez)

td = 0;

for i = 1 : length(Path_bez)-1
    
    td = td + norm( Path_bez(i+1,:) - Path_bez(i,:) );
    
end

total_distance = td;

end