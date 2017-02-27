directory_prefix = 'advanced_robotics';
abtc = ['1', '2', '3', '4', '5', '6', '7', '8'];

% Initialize the cell arrays to hold the filenames
f_fe = cell(size(abtc));
f_re = cell(size(abtc));
f_ic_st = cell(size(abtc));
f_ic_oc = cell(size(abtc));
f_tl = cell(size(abtc));

% Set the filenames
for i=1:size(abtc,2)
    f_prefix = fullfile(directory_prefix, abtc(i));
    
    f_fe(i) = {fullfile(f_prefix, 'feasible.txt')};
    f_re(i) = {fullfile(f_prefix, 'reached_goal.txt')};
    f_ic_st = {fullfile(f_prefix, 'ic_stuck.txt')};
    f_ic_oc = {fullfile(f_prefix, 'ic_occurred.txt')};
    f_tl = {fullfile(f_prefix, 'time_left.txt')};
end


data_fe = importdata(f_fe{1});
    
% Go through each f_fe
for i=1:size(f_fe,2)
    data_fe(i) = importdata(f_fe{i});
end


fe = importdata(strcat('1/', f_fe));
fe = [fe; importdata(strcat('2/', f_fe))];
fe = [fe; importdata(strcat('3/', f_fe))];
fe = [fe; importdata(strcat('4/', f_fe))];
fe = [fe; importdata(strcat('5/', f_fe))];
fe = [fe; importdata(strcat('6/', f_fe))];
fe = [fe; importdata(strcat('7/', f_fe))];
fe = [fe; importdata(strcat('8/', f_fe))];

tl = importdata(strcat('1/', f_tl));
tl = [tl; importdata(strcat('2/', f_tl))];
tl = [tl; importdata(strcat('3/', f_tl))];
tl = [tl; importdata(strcat('4/', f_tl))];
tl = [tl; importdata(strcat('5/', f_tl))];
tl = [tl; importdata(strcat('6/', f_tl))];
tl = [tl; importdata(strcat('7/', f_tl))];
tl = [tl; importdata(strcat('8/', f_tl))];

re = importdata(strcat('1/', f_re));
re = [re; importdata(strcat('2/', f_re))];
re = [re; importdata(strcat('3/', f_re))];
re = [re; importdata(strcat('4/', f_re))];
re = [re; importdata(strcat('5/', f_re))];
re = [re; importdata(strcat('6/', f_re))];
re = [re; importdata(strcat('7/', f_re))];
re = [re; importdata(strcat('8/', f_re))];

ic_st = importdata(strcat('1/', f_ic_st));
ic_st = [ic_st; importdata(strcat('2/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('3/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('4/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('5/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('6/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('7/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('8/', f_ic_st))];


ic_oc = importdata(strcat('1/', f_ic_oc));
ic_oc = [ic_oc; importdata(strcat('2/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('3/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('4/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('5/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('6/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('7/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('8/', f_ic_oc))];
