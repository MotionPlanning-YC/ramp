directory_prefix = 'advanced_robotics';
abtc = ['1', '2', '3', '4', '5', '6', '7', '8', '9'];

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
    f_ic_st(i) = {fullfile(f_prefix, 'ic_stuck.txt')};
    f_ic_oc(i) = {fullfile(f_prefix, 'ic_occurred.txt')};
    f_tl(i) = {fullfile(f_prefix, 'time_left.txt')};
end

% Initialize data arrays
data_fe = zeros(100,8);
data_re = zeros(100,8);
data_ic_st = zeros(100,8);

% The # of values for Time left is equal to how many times the robot
% ended in a feasible trajectory so it will be different for each set 
% of test cases. Make a cell array instead of an NxM array
data_tl = cell(8,1);
    
% Go through each filename (use f_fe as counter for the loop)
% and get the data
for i=1:size(f_fe,2)
    data_fe(1:100,i) = importdata(f_fe{i});
    data_re(1:100,i) = importdata(f_re{i});
    data_ic_st(1:100,i) = importdata(f_ic_st{i});
    
    data_tl{i} = importdata(f_tl{i});
end


% Strip Time Left values that are greater than 1000
% These values are from when the robot reaches the goal so the trajectory's
% last point is -1, which become huge numbers when using uints
for i=1:size(data_tl,1)
    data_tl{i} = data_tl{i}(data_tl{i} < 1000);
end

% Initialize percentage arrays
perc_fe = zeros(1,8, 'double');
perc_re = zeros(1,8, 'double');
perc_ic_st = zeros(1,8, 'double');

% Get percentages
for i=1:size(abtc,2)
    s = sum(data_fe(:,i) == 1);
    perc_fe(i) = s / 100.0;
    
    s = sum(data_re(:,i) == 1);
    perc_re(i) = s / 100.0;
    
    s = sum(data_ic_st(:,i) == 1);
    perc_ic_st(i) = s / 100.0;
    
end

% Time-left is a little different...
% Get histogram and average values for each ABTC
mean_tl = zeros(size(abtc));
for i=1:size(data_tl,1)
    d = fitdist(data_tl{i}, 'Normal');
    mean_tl(i) = d.mu;
end



% Put everything together
data_all_fe = reshape(data_fe, [], 1);
data_all_re = reshape(data_re, [], 1);
data_all_ic_st = reshape(data_ic_st, [], 1);
data_all_tl = data_tl{1};

for i=2:size(data_tl,1)
    data_all_tl = [data_all_tl; data_tl{i}];
end


% Get percentages
s = sum(data_all_fe(:) == 1);
perc_all_fe = s / size(data_all_fe,1);

s = sum(data_all_re(:) == 1);
perc_all_re = s / size(data_all_re,1);

s = sum(data_all_ic_st(:) == 1);
perc_all_ic_st = s / size(data_all_ic_st,1);
    
nbins = 50;
tl_hist = hist(data_all_tl, nbins);
bar(tl_hist);
xlabel('Seconds left in trajectory');

perc_all = [perc_all_fe-perc_all_re-perc_all_ic_st perc_all_re 1-perc_all_fe perc_all_ic_st];
perc_values = {sprintf('%.0f%%', 100*perc_all(1)); sprintf('%.0f%%', 100*perc_all(2)); sprintf('%.0f%%', 100*perc_all(3)); sprintf('%.0f%%', 100*perc_all(4))};

strs = {strcat('Goal: ', perc_values{1}) ; strcat('Feasible: ', perc_values{2}) ; strcat('Infeasible: ', perc_values{3}); strcat('I.C. (stuck): ', perc_values{4})};

p = pie(perc_all, strs);
set(p(2:2:8),'FontSize',12);
set(gcf, 'Position', [10 10 500 500]);



% Old scripts
% tl = importdata(strcat('1/', f_tl));
% tl = [tl; importdata(strcat('2/', f_tl))];
% tl = [tl; importdata(strcat('3/', f_tl))];
% tl = [tl; importdata(strcat('4/', f_tl))];
% tl = [tl; importdata(strcat('5/', f_tl))];
% tl = [tl; importdata(strcat('6/', f_tl))];
% tl = [tl; importdata(strcat('7/', f_tl))];
% tl = [tl; importdata(strcat('8/', f_tl))];
% 
% re = importdata(strcat('1/', f_re));
% re = [re; importdata(strcat('2/', f_re))];
% re = [re; importdata(strcat('3/', f_re))];
% re = [re; importdata(strcat('4/', f_re))];
% re = [re; importdata(strcat('5/', f_re))];
% re = [re; importdata(strcat('6/', f_re))];
% re = [re; importdata(strcat('7/', f_re))];
% re = [re; importdata(strcat('8/', f_re))];
% 
% ic_st = importdata(strcat('1/', f_ic_st));
% ic_st = [ic_st; importdata(strcat('2/', f_ic_st))];
% ic_st = [ic_st; importdata(strcat('3/', f_ic_st))];
% ic_st = [ic_st; importdata(strcat('4/', f_ic_st))];
% ic_st = [ic_st; importdata(strcat('5/', f_ic_st))];
% ic_st = [ic_st; importdata(strcat('6/', f_ic_st))];
% ic_st = [ic_st; importdata(strcat('7/', f_ic_st))];
% ic_st = [ic_st; importdata(strcat('8/', f_ic_st))];
% 
% 
% ic_oc = importdata(strcat('1/', f_ic_oc));
% ic_oc = [ic_oc; importdata(strcat('2/', f_ic_oc))];
% ic_oc = [ic_oc; importdata(strcat('3/', f_ic_oc))];
% ic_oc = [ic_oc; importdata(strcat('4/', f_ic_oc))];
% ic_oc = [ic_oc; importdata(strcat('5/', f_ic_oc))];
% ic_oc = [ic_oc; importdata(strcat('6/', f_ic_oc))];
% ic_oc = [ic_oc; importdata(strcat('7/', f_ic_oc))];
% ic_oc = [ic_oc; importdata(strcat('8/', f_ic_oc))];
