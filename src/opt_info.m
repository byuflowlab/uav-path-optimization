
% minimize distance
%{
rng(8); %54/4/3
if one_path == 1
    num_path = 16;
    ms_i = 1;
    get_bez_points = @rng8;
end
%}

%{
%rng(59); %4,3,54
if one_path == 1
    num_path = 16;
    ms_i = 1;
    get_bez_points = @rng59;
end
%}

%{
%rng(4); %4,3,49
if one_path == 1
    num_path = 15;
    ms_i = 1;
    get_bez_points = @rng4_d;
end
%}

%minimize time

%{
%rng(58); %4,3,54
if one_path == 1
    ms_i = 1;
    get_bez_points = @rng58_t;
    num_path = 14;
end
%}

%{
%rng(4); %4,3,49
if one_path == 1
    ms_i = 1;
    get_bez_points = @rng4_t;
    num_path = 9;
end
%}

%minimize energy
%{
%rng(4); %4,3,49
if one_path == 1
    ms_i = 1;
    get_bez_points = @rng4_e;
    num_path = 12;
end
%}