

clc
clear
close all 
format long
%% Creating notch box plot and table for generated data
% number of epucks
n = [10 30 50 70 100 120];

%% For Power Failure

% reading the data from txt file and storing them
TP1 = dlmread('PowerFailureData/powerfailure_10epucks.txt');
TP2 = dlmread('PowerFailureData/powerfailure_30epucks.txt');
TP3 = dlmread('PowerFailureData/powerfailure_50epucks.txt');
TP4 = dlmread('PowerFailureData/powerfailure_70epucks.txt');
TP5 = dlmread('PowerFailureData/powerfailure_100epucks.txt');
TP6 = dlmread('PowerFailureData/powerfailure_120epucks.txt');

% ploting, labeling and saving the notch boxplot
tempfig1 = figure();
boxplot([TP1,TP2,TP3,TP4,TP5,TP6],'Notch','on','Labels',n)
title('Comparing the time to reach goal of different number of epucks')
xlabel('number of epucks')
ylabel('time taken to reach goal')
saveas(tempfig1,'saved_figures/powerfailure_plot.fig')

% calculating mean and standard deviation
stand_dev_power = [ std(TP1) std(TP2) std(TP3) std(TP4) std(TP5) std(TP6)];
mean_power = [ mean(TP1)  mean(TP2) mean(TP3) mean(TP4) mean(TP5) mean(TP6)];

% printing the results in table
tab_power_failure = table;
tab_power_failure.number_of_epucks =  n';
tab_power_failure. st_dev = stand_dev_power';
tab_power_failure.mean = mean_power'




%% For Sensor Failure

% reading the data from txt file and storing them
TS1 = dlmread('SensorFailureData/sensorfailure_10epucks.txt');
TS2 = dlmread('SensorFailureData/sensorfailure_30epucks.txt');
TS3 = dlmread('SensorFailureData/sensorfailure_50epucks.txt');
TS4 = dlmread('SensorFailureData/sensorfailure_70epucks.txt');
TS5 = dlmread('SensorFailureData/sensorfailure_100epucks.txt');
TS6 = dlmread('SensorFailureData/sensorfailure_120epucks.txt');

% ploting, labeling and saving the notch boxplot
tempfig2 = figure();
boxplot([TS1,TS2,TS3,TS4,TS5,TS6],'Notch','on','Labels',n)
title('Comparing the time to reach goal of different number of epucks')
xlabel('number of epucks')
ylabel('time taken to reach goal')
saveas(tempfig2,'saved_figures/sensorfailure_plot.fig')

% calculating mean and standard deviation
stand_dev_sensor = [ std(TS1) std(TS2) std(TS3) std(TS4) std(TS5) std(TS6)];
time_mean_sensor = [ mean(TS1)  mean(TS2) mean(TS3) mean(TS4) mean(TS5) mean(TS6)];

% printing the results in table
tab_sensor_failure = table;
tab_sensor_failure.number_of_epucks =  n';
tab_sensor_failure. st_dev = stand_dev_sensor';
tab_sensor_failure.mean = time_mean_sensor'

%% For Motor Failure


TM1 = dlmread('MotorFailureData/motorfailure_10epucks.txt');
TM2 = dlmread('MotorFailureData/motorfailure_30epucks.txt');
TM3 = dlmread('MotorFailureData/motorfailure_50epucks.txt');
TM4 = dlmread('MotorFailureData/motorfailure_70epucks.txt');
TM5 = dlmread('MotorFailureData/motorfailure_100epucks.txt');
TM6 = dlmread('MotorFailureData/motorfailure_120epucks.txt');

% ploting, labeling and saving the notch boxplot
tempfig3 = figure();
boxplot([TM1,TM2,TM3,TM4,TM5,TM6],'Notch','on','Labels',n)
title('Comparing the time to reach goal of different number of epucks')
xlabel('number of epucks')
ylabel('time taken to reach goal')
saveas(tempfig2,'saved_figures/motorfailure_plot.fig')

% calculating mean and standard deviation
stand_dev_motor = [ std(TM1) std(TM2) std(TM3) std(TM4) std(TM5) std(TM6)];
time_mean_motor = [ mean(TM1)  mean(TM2) mean(TM3) mean(TM4) mean(TM5) mean(TM6)];

% printing the results in table
tab_motor_failure = table;
tab_motor_failure.number_of_epucks =  n';
tab_motor_failure. st_dev = stand_dev_motor';
tab_motor_failure.mean = time_mean_motor'


