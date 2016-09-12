%http://cafefoundation.org/v2/pdf_tech/MPG.engines/AIAA.1980.1847.B.H.Carson.pdf
clear; clc; close all;

%Check to see how cruise efficiency is affected by velocity

%parameter values
rho = 1.225; %air density
f = .2;   %equivalent parasite area
W = 10; %weight of aircraft
b = .20;   %span
e = 0.9; %Oswald's efficiency factor

%velocity
v = 0.01 : 0.1 : 30;

%Defined in paper (2nd column, page 2)
A = rho*f/(2*W);
B = 2*W/(rho*b^2*pi*e);

%vehicle specific power (2nd column, page 4)
%epsilon = 1;
% https://en.wikipedia.org/wiki/Vehicle-specific_power
% http://cires1.colorado.edu/jimenez/Papers/Jimenez_VSP_9thCRC_99_final.pdf

%defined in paper (1st column, page 5)
% for i = 1 : length(v)
%

%cruise efficiency
%     k(i) = epsilon(i)/v(i);
%
% end
% for i = 1 : length(v)
%
%     C(i) = 0.57*k(i)*(A^3*B)^(-1/4);
%
% end

%calculate l_d

for i = 1 : length(v)
    
    d_l(i) = A*v(i)^2 + B/v(i)^2;
    
    l_d(i) = d_l(i)^(-1);
    
end

figure(1);
hold on;

    set(gca,'FontSize',20)
    set(gca,'FontSize',20)

plot(v,d_l,'LineWidth',2);
xlabel('UAV Velocity (m/s)','fontsize',18);
ylabel('Drag (N)','fontsize',18);
%title('Drag vs. UAV Speed');
ylim([0 40]);
hold off;
%plot propulsive efficiency
V = 0.01 : 0.01 : 30;

a = -0.0024;
b = 0.084;
c = -0.9;
d = 3.6;

eta = zeros(length(V),1);

for i = 1 : length(V)
    
    if V(i) < 10
        
        eta(i) = V(i)*0.06;
        
    elseif V(i) < 20
        
        eta(i) = a*V(i)^3 + b*V(i)^2 + c*V(i) + d;
        
    else
        
        eta(i) = 0.00001;
        
    end
end

figure(2);
hold on;

    set(gca,'FontSize',20)
    set(gca,'FontSize',20)

plot(V,eta,'LineWidth',2);
%title('Efficiency vs. UAV Speed');
xlim([0 21]);
ylim([0 1]);
xlabel('UAV Velocity (m/s)','fontsize',18);
ylabel('Overall Propulsive Efficiency','fontsize',18);
hold off;

%-----------------------------------------%

for i = 1 : length(V)
    
    opt(i) = d_l(i)/eta(i);
    
end


figure(3);
hold on;

    set(gca,'FontSize',20)
    set(gca,'FontSize',20)

plot(V,opt);
ylim([0 20]);
xlim([5 20]);
hold off;