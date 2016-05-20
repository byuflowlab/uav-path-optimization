function eta = calc_eff(V)
%input for function is V (proportional to length of segment), output is eta

a = -0.0024;
b = 0.084;
c = -0.9;
d = 3.6;


if V < 10
    
    eta = V*0.06;
    
elseif V < 20
    
    eta = a*V^3 + b*V^2 + c*V + d;
    
else
    
    eta = 0.00001;
    
end

%end

%plot(V,eta);
%ylim([0 1]);
end
