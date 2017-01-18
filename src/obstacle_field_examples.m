
% --------- Challenging Obstacle Field Examples ------ %

%rng(1); %54/4/3 -pretty easy; 50/4/3 - more interesting
%rng(1); %40, 3.5/3; 50,0 and 2,7 - goes below obstacles
%rng(1); %49/4/3 %similar 3 comparison (2)
%rng(2); %50,3.75,3 - works for num_path = 3;
%rng(2); %52,3.5,3 -easy start, difficult middle portion
%rng(3); %50,4,3
%rng(4); %49,4,3
%rng(5); %40, 3/3; 50,0 and 2,6.5 - goes above
%rng(5); %49/4/3 - similar 3 comparison (1)
%rng(6); %50,4,3 - use for comparison of removing unconverged solutions
%rng(7); %50,4,3 %use this for opt_d, delta_t = 0.1, for num_path comparison
%rng(8); %54, 4, 3 %good example for ms=1 fails ms=3 succeeds
%rng(9); %50,4,3 - num_path comparison
%rng(11); %55,4,3 - used for the three comparison
%rng(18);  %4,3,54
%rng(22); %50/5/3 - difficult start, straight forward finish; for delta_t=0.2, fails if you don't remove bad solutions
%rng(51); %3.5,3,50 - optimized finish comparison
%rng(59); %3,3, 55   - use for comparison of three objective functions
%rng(59); %4,3,54
%rng(60); %3.5, 3, 40 or 4/3/50
%rng(101); %50/54,4,3 -difficult start, works for delta_t = 0.1

%-------ucur results-----------%

%first static obstacle avoidance (show steps)
%rng(2); %52,3.5,3,delta_t=0.1,ms=3,num_path=3

%additional static obstacle avoidance (don't show steps)
%rng(6); %50,4,3 - static2
%rng(18);  %4,3,54 - static3
%rng(59); %54/4/3 - static4, all of these are optimize_time

%dynamic obstacle avoidance
%rng(2); %3,3.5,1; dyn_case = 1; optimize time

%dynamic obstacle avoidance comparison
%rng(2); %3,3.5,1; dyn_case = 5; delta_t = 0.1

%---------paper results----------%

% method1.eps = %rng(3); %50/4/3 (figure(5))
% BezCurveCloseUp.eps %rng(7); %50/4/3
% calc_fig.eps = %rng(3); %50/4/3 (figure(5))
% fail.eps = %rng(8); %50/4/3; figure(2), ms_i = 1, ms_i = 5
% pass.eps = %rng(8); %50/4/3; figure(2), ms_i = 1, ms_i = 5
% compare.eps = % rng(7); % 50/4/3



%YOUTUBE VIDEO
%rng(3); %50/4/3 - 5/17a - time
%rng(10); %50/4/3 -5/18a
%rng(1); %50/4/3 - 5/18b - distance
%rng(5); %52/4/3 - 5/19a - distance
%rng(6); % 52/3.75/3 - 5/20a - distance

%Methodology
% ? %rng(1); %50/4/3
%rng(3); %50/4/3 (figure(5))

%multi start approach
%rng(8); %50/4/3; figure(2), ms_i = 1, ms_i = 5

%first static obstacle avoidance (show steps)
rng(2); %52,3.5,3,delta_t=0.1,ms=3,num_path=3

%DOA
% dyn_case = 6;

%DOA_C
% dyn_case = 5;

% EU vs. Time vs. Distance
%rng(60); %50/4/3 % 1
%rng(59); %54/4/3 % 2

% Path Compare to optimal
%rng(3); %47/4/3

%-----------------------------%
%color change representing speed
%rng(2); %50/3.75/3 - good one
%rng(8); %4/3/54 - good for all three
%rng(1); %4/3/57

%SDOA
%rng(2); %44/4/3, dyn_case = 7