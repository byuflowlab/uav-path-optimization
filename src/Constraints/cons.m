function [c, ceq, gc, gceq] = cons(xi)

global num_path cons_grad;

c = [];
ceq = [];
gc = [];
gceq = [];

[c, ceq] = calc_cons(xi);

if cons_grad == 1
    %calculate c gradients using complex step
    h = 10^(-20);
    gc = zeros(num_path*4,length(c));
    
    for i = 1 : num_path*2
        
        for j = 1 : 2
            
            xi(i,j) = xi(i,j) + 1i*h;
            
            [ci,~] = calc_cons(xi);
            
            gc(2*num_path*(j-1)+i,:) = imag(ci)/h;
            
            xi(i,j) = xi(i,j) - 1i*h;
            
        end
        
    end
  
    %calculate ceq gradients using complex step
 
    gceq = zeros(num_path*4,length(ceq));
    
    for i = 1 : num_path*2
        
        for j = 1 : 2
            
            xi(i,j) = xi(i,j) + 1i*h;
            
            [~,ceqi] = calc_cons(xi);
            
            %gceq(2*num_path*(j-1)+i,l) = imag(ceqi(l))/h;
            gceq(2*num_path*(j-1)+i,:) = imag(ceqi)/h;
            
            xi(i,j) = xi(i,j) - 1i*h;
            
        end
        
    end
    

end

end