close all;

%paredo front for 

%rng(9), 48/4/3
% 2 - 5 num_path
%time = [23 50 133 243 318 376];
%dis = [169.06 156.4 151.9 149.05 148.96 148.816];

%rng(9), 50/4/3
% ms = 3
time = [29.4 81.8 175.6 375.8 516.2 778.43];
dis = [ 160.0276
  147.2014
  146.5553
  146.2599
  146.2529
  146.2064 ];

figure(1);
hold on;
plot(time(1),dis(1),'yx');
plot(time(2),dis(2),'mx');
plot(time(3),dis(3),'cx');
plot(time(4),dis(4),'rx');
plot(time(5),dis(5),'gx');
plot(time(6),dis(6),'bx');
legend('1 segment planned','2 segments planned','3 segments planned','4 segments planned',...
       '5 segments planned','6 segments planned');
xlabel('Algorithm Completion Time (s)');
ylabel('Total Distance (m)');
xlim([0 800]);
ylim([145 165]);
title('Number of Planned Path Segment Comparison');
hold off;