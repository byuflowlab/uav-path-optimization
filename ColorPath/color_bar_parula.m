clear; clc; close all;
x = 1:10;
y = 2: 2 : 20;

plot(x,y);
colorbar('southoutside','Ticks',[0,0.20,0.4,0.6,0.8,1],'TickLabels',{'V_{min}, 10 m/s','11 m/s','12 m/s','13 m/s','14 m/s','V_{max},15 m/s'},'fontsize',14);

