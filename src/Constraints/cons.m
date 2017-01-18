function [c, ceq, gc, gceq] = cons(xi)

global num_path cons_grad acg;

c = [];
ceq = [];
gc = [];
gceq = [];

if acg == 0
    [c, ceq] = calc_cons(xi);
end

if cons_grad == 1 && acg == 0
    
    h = 10^(-20);
    gc = zeros(num_path*4,length(c));
    gceq = zeros(num_path*4,length(ceq));
    
    for i = 1 : num_path*2
        
        for j = 1 : 2
            
            xi(i,j) = xi(i,j) + 1i*h;
            
            %calculate c gradients using complex step
            [ci,~] = calc_cons(xi);
            
            gc(2*num_path*(j-1)+i,:) = imag(ci)/h;
            
            %calculate ceq gradients using complex step
            [~,ceqi] = calc_cons(xi);
            
            gceq(2*num_path*(j-1)+i,:) = imag(ceqi)/h;
            
            xi(i,j) = xi(i,j) - 1i*h;
            
            
        end
        
    end
    
elseif cons_grad == 1 && acg == 1
    
    [c, ceq, gc, gceq] = calc_cons_acg(xi);
    
%     %curvature constraints
%     curvature_c = curvature_cons(xi);
%     
%     c = [c curvature_c];
    
end

%gradients of curvature constraints calculated using finite difference
% dh = 10.0^(-6.0);
% 
% curvature_gc = zeros(4*num_path,27);
% 
% for i = 1 : num_path*2
%     
%     for j = 1 : 2
%         
%         xi(i,j) = xi(i,j) + dh;
%         
%         c_x_plus_h = curvature_cons(xi);
%         
%         xi(i,j) = xi(i,j) - dh;
%         
%         c_x = curvature_cons(xi);
%         
%         curvature_gc(2*num_path*(j-1)+i,:) = (c_x_plus_h - c_x)/dh;
%         
%         
%     end
%     
% end
% 
% gc = [gc curvature_gc];


c = real(c);

end