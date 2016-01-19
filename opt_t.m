function [e] = opt_t(xi)

global xf; %final location of object
global num_path;

x_last = real(xi(2*num_path,1)); y_last = real(xi(2*num_path,2));

%calculate distance from final path segment's end point and final
%destination
e = ( (xf(1) - x_last)^2 + (xf(2) - y_last)^2 )^0.5;

end