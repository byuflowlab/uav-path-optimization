function  [num pos size vel] = dyn_case(case_num)

if case_num == 1 %one big obstacle coming from final destination, toward initial position
    
    num = 1;
    pos = [100,100];
    size = 15;
    vel = [-8,-8];
    
    
elseif case_num == 2 %two obstacles, one big/small, coming from final destination
    
    num = 2;
    pos = [100,100; 90,90];
    size = [15;8];
    vel = [-5,-5;-18,-18];
    
elseif case_num == 3 %two offset obstacles from final destination
    
    num = 2;
    pos = [100,90; 70,80];
    size = [10;10];
    vel = [-8,-8;-8,-8];
    
elseif case_num == 4 %obstacle coming across face of UAV
    num = 1;
    pos = [75,25];
    size = 10;
    vel = [-3.12,3.12];
    
elseif case_num == 5 % 2 obstacles coming acros face of UAV, different directions
    
   num = 2;
   pos = [75,25; 25,75];
   size = [10;10];
   %vel = [-6,6; 6,-6];
   %vel = [-4.5,4.5; 4.5,-4.5]; 
   vel = [-5,5; 5,-5]; 
   
elseif case_num == 6 % multiple obstacles coming acros face of UAV, same direction
    
   num = 3;
   pos = [75,25; 95,25; 75,5];
   size = [7;7;7];
   vel = [-6.25,6.25; -6.25,6.25;  -6.25,6.25];
   
elseif case_num == 7
    
    num = 1;
    pos = [34,93];
    size =  6;
    vel = [4.5,-7];
   
end



end