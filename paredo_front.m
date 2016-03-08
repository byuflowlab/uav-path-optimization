close all;

%paredo front for 

%rng(9), 48/4/3
% 2 - 5 num_path
time = [23 50 133 243 318 376];
dis = [169.06 156.4 151.9 149.05 148.96 148.816];

%rng(9), 50/4/3
% ms = 3
% time = [29.4 81.8 175.6 375.8 516.2 778.43];
% dis = [ 160.0276
%   147.2014
%   146.5553
%   146.2599
%   146.2529
%   146.2064 ];

%rng(3), 50/4/3
% ms = 3;
%time = [45 93 127 204 344 596];
%dis = [172.253923641734;170.410239996284;149.849643976224;148.351190385225;148.053963065152;147.600466512524];

figure(1);
hold on;
plot(time(1),dis(1),'bo');
plot(time(2),dis(2),'bs');
plot(time(3),dis(3),'bd');
plot(time(4),dis(4),'b^');
plot(time(5),dis(5),'b<');
plot(time(6),dis(6),'b>');
legend('1 segment planned','2 segments planned','3 segments planned','4 segments planned',...
       '5 segments planned','6 segments planned');
xlabel('Algorithm Completion Time (s)');
ylabel('Total Distance (m)');
%xlim([0 650]);
%ylim([145 175]);
xlim([0 400]);
ylim([148 170]);
title('Number of Planned Path Segment Comparison');
hold off;