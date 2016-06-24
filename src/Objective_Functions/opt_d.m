function [f,g] = opt_d(xi)

global num_path obj_grad ag;

f = [];
g = [];

%calculate value of objective function
[f,g] = calc_f_opt_d(xi);

if obj_grad == 1 && ag == 0
    %calculate gradients using complex step
    h = 10^(-20);
    g = zeros(num_path*2,2);
    
    for i = 1 : num_path*2
        for j = 1 : 2
            
            xi(i,j) = xi(i,j) + 1i*h;
            
            fi = calc_f_opt_d(xi);

            g(i,j) = imag(fi)/h;
            
            xi(i,j) = xi(i,j) - 1i*h;
            
        end
        
    end
    
end

end