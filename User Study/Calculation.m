clc
clear
close all

%% Variable Setup
Jackal_TLX = xlsread('Evaluation.xlsx', 'B4:G20');
Jackal_SUS = xlsread('Evaluation.xlsx', 'H4:Q20');

Panda_TLX = xlsread('Evaluation.xlsx', 'R4:W20');
Panda_SUS = xlsread('Evaluation.xlsx', 'X4:AG20');

TLX_weights = xlsread('Evaluation.xlsx', 'AH4:AM20');

%Calculate Means
Jackal_SUS_mean = mean(Jackal_SUS);
Panda_SUS_mean = mean(Panda_SUS);


%% Plots
tlx_names = {'Mental'; 'Physical'; 'Temporal' ; 'Performance' ; 'Effort' ; 'Frustration'};

figure(1)
subplot(1,3,1)
boxplot(Jackal_TLX)
set(gca,'xtick',[1:6],'xticklabel',tlx_names)
xtickangle(90)
ylim([1 20])
title('Jackal TLX')

subplot(1,3,2)
boxplot(Panda_TLX)
set(gca,'xtick',[1:6],'xticklabel',tlx_names)
xtickangle(90)
ylim([1 20])
title('Panda TLX')

subplot(1,3,3)
boxplot(TLX_weights)
set(gca,'xtick',[1:6],'xticklabel',tlx_names)
xtickangle(90)
set(gca,'ytick',[1:5])
ylim([0 5])
title('TLX Weights')

saveas(gcf,'TLX.png')

figure(2)
subplot(1,2,1)
boxplot(Jackal_SUS)
set(gca,'ytick',[1:5])
ylim([1 5])
title('Jackal SUS')

subplot(1,2,2)
boxplot(Panda_SUS)
ylim([1 5])
set(gca,'ytick',[1:5])
title('Panda SUS')

saveas(gcf,'SUS.png')


%% Calculate SUS Score
Jackal_SUS_odd = (Jackal_SUS_mean(1:2:end))-1;
Jackal_SUS_even = 5-(Jackal_SUS_mean(2:2:end));

Jackal_SUS_Score = (sum(Jackal_SUS_odd)+sum(Jackal_SUS_even))*2.5

Panda_SUS_odd = (Panda_SUS_mean(1:2:end))-1;
Panda_SUS_even = 5-(Panda_SUS_mean(2:2:end));

Panda_SUS_Score = (sum(Panda_SUS_odd)+sum(Panda_SUS_even))*2.5

%% Calculate TLX Score
[sz1, ~] = size(TLX_weights);
for i = 1:sz1
   Jackal_TLX_Scores(i) = (Jackal_TLX(i,:)*TLX_weights(i,:)')/15; 
   Panda_TLX_Scores(i) = (Panda_TLX(i,:)*TLX_weights(i,:)')/15;
end

Jackal_TLX_Scores_mean = mean(Jackal_TLX_Scores)
Panda_TLX_Scores_mean = mean(Panda_TLX_Scores)

%% TLX Range
Min_range = [1 1 1 1 1 1];
Max_range = [20 20 20 20 20 20];

basic_weights = [0 1 2 3 4 5];
min_score = (Min_range*basic_weights')/15
max_score = (Max_range*basic_weights')/15


