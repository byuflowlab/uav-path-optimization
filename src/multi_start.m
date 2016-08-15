function [x_guess] = multi_start(ms_i)

su = 0;

%global variables
global num_path x0 step_max step_min x_next start;

x_guess = zeros(2*num_path,2,ms_i);

for i = 1 : ms_i
    
    for j = 1 : 2*num_path
        
        %different starting paths for different cases
        
        if i == 1 %top right diagonal
            
            if num_path > 1
                
                if start == 1 %
                    
                    if j >= 2*num_path-1
                        x_guess(j,:,i) = [x_guess(2*num_path-2,1,i) + 0.25*(j-(2*num_path-2))*step_max, x_guess(2*num_path-2,2,1) + 0.25*(j-(2*num_path-2))*step_max + su];
                    else
                        x_guess(j,:,i) = [x_next(j+2,1),x_next(j+2,2) + su];
                    end
                    
                else
                    x_guess(j,:,i) = [x0(1) + 0.25*j*step_max, x0(2) + 0.25*j*step_max + su];
                end
                
            else
                
                x_guess(j,:,i) = [x0(1) + 0.25*j*step_max, x0(2) + 0.25*j*step_max + su];
                
            end
            
        elseif i == 2 %right straight
            x_guess(j,:,i) = [x0(1) + 0.25*j*step_max, x0(2) + 0.25*(j-j^2/(num_path*4))*step_max];
            
        elseif i == 3 %top straight
            x_guess(j,:,i) = [x0(1) + 0.25*(j-j^2/(num_path*4))*step_max, x0(2) + 0.25*j*step_max];       
            
        elseif i == 4 %to the right, slightly down
            x_guess(j,:,i) = [x0(1)+0.25*j*step_max, x0(2)-0.125*j*step_max];
            %x_guess(j,:,i) = [x0(1) + 0.125*j*step_max, x0(2) + 0.125*(j-j^2/(num_path*4))*step_max];
            
        elseif i == 5 %to the top, slightly left
            x_guess(j,:,i) = [x0(1)-0.125*j*step_max, x0(2)+0.25*j*step_max];
            %x_guess(j,:,i) = [x0(1) + 0.125*(j-j^2/(num_path*4))*step_max, x0(2) + 0.125*j*step_max];
            
        elseif i == 6 %to the right, slightly down
            x_guess(j,:,i) = [x0(1)+0.125*j*step_max, x0(2)+0.0625*j*step_max];
            
        elseif i == 7 %to the top, slightly left
            x_guess(j,:,i) = [x0(1)+0.0625*j*step_max, x0(2)+0.125*j*step_max];
            
        else
            break %outside of range, need to initialize more guesses (do randomly maybe?)
            
        end
        
    end
    
end