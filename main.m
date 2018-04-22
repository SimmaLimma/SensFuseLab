%% General TODO:s
% Check if comments refer to range and not time

%% Init
clear all
close all
load('testNew.mat')

% Calculating range [meter] from time [second]. 343 m/s is speed of sound
rphat = tphat * 343;



%% Correction tphat, given by Hendeby

%tphat = SFlabFindPulseTimes(recBuffer, recChanList, speakers, sampRate, pulseFreq, pulseWidth, npt, time, plotOn, [], false);

%% Estimating measurement noise to estimate R
meanrphat = mean(rphat')';

%TODO: Is code below redudant?
e = zeros(size(rphat));
% e is measurement error for position estimation
for mic = 1:8
    e(:,mic) = rphat(:,mic) - meanrphat;
end


%TODO: Calculate e with pairwise diff directly, and check if same result
%Calculating e(i,j) = e(i) - e(j)
rphatPair = zeros(88,28);

rowInd = 1;
for i = 1:7
    for j = i+1:8
        rphatPair(:,rowInd) = rphat(:,i) - rphat(:,j);
        rowInd = rowInd +1;
    end
end


%TODO: Try to replace eTmp with tphat(:,mic) - meantphat
%Calculating e(i,j) = e(i) - e(j)
ePair = zeros(88,28);

rowInd = 1;
for i = 1:7
    for j = i+1:8
        ePair(:,rowInd) = e(:,i) - e(:,j);
        rowInd = rowInd +1;
    end
end


% Normalized histogram of e (code given by lab pm)
%[N, l] = hist(e ,20);
%Wb=l(2)-l(1); % Bin  width
%Ny = length(e); % Nr of  samples
%figure()
%bar(l, N/(Ny*Wb)); title('Normalized histogram of error')

%% Comparison of noise
pe = ndist(mean(ePair)',diag(var(ePair))); 

%TODO: Make plot work. Maybe use a for loop?
%Uncomment for distribution of pe
%figure()
%plot(pe); title('Estimation of PDF of error')

%% Creating sensor model
%Model for config 1
sm1 = exsensor('tdoa2',8,1,2);

%Model for config 2
sm2 = exsensor('tdoa2',8,1,2);

%Start position of robot (at time instance 0)
startPos = [0.383; 0.095];

%TODO: Make init func
% x,y value for each sensor in config 1
micPos1 = [[0, 0.1416*0];
           [0, 0.1416*1];
           [0, 0.1416*2];
           [0, 0.1416*3];
           [0, 0.1416*4];
           [0, 0.1416*5];
           [0, 0.1416*6];
           [0, 0.1416*7]]';
    
% x,y value for each sensor in config 2
micPos2 = [[0.4073*0, 0.3303*1];
           [0.4073*0, 0.3303*2];
           [0.4073*1, 0.3303*3];
           [0.4073*2, 0.3303*3];
           [0.4073*3, 0.3303*2];
           [0.4073*3, 0.3303*1];
           [0.4073*2, 0.3303*0];
           [0.4073*1,0.3303*0]]';

%Setting sensor positions according to setup
sm1.th = micPos1(:);
sm2.th = micPos2(:);

%Setting inital position of the robot
sm1.x0 = startPos;
sm2.x0 = startPos;

% Setting PDF for measurement noise
% TODO: Try out cov instead of diag(var(ePair)). I think mathematically
% pretty much same, since mean "takes out eachother"
sm1.pe = diag(var(ePair));
sm2.pe = diag(var(ePair));

%Plotting the configs
%figure(1)
%sm1.plot
%xlim([0 1.23])
%ylim([0 1])

%figure(2)
%sm2.plot
%xlim([0 1.23])
%ylim([0 1])
%% Analysis of model

% TODO: Code below redudant?
% Creating SIG object from measurments
sigrphat = sig(rphat,sampRate);
 
% CRLB analysis. Uncomment for plot
% TODO: Can we use another approach than TDOA2? (Since Hendeby said we had to use TDOA1 somewhere)
%figure(1)
%hold on
%Empty y, limits set according to lab setup and calc of RMSE
%crlb2(sm1, [], 0:0.1:1.3, 0:0.1:1, [1 2], 'rmse'); 
%title('CRLB analysis of config 1')

%figure(2)
%hold on
%Empty y, limits set according to lab setup and calc of RMSE
%crlb2(sm2, [], 0:0.1:1.3, 0:0.1:1, [1 2], 'rmse');  
%title('CRLB analysis of config 2')

%hold off

%% Localization
% Config 2 is choosen as preferred config (due to config analysis)

% TODO: Make this into a function

%Load config 1
load('testconfig2.mat')

% Calculating range [meter] from time [second].
rphatC2 = 343 * tphat;

% Location estimation with pairwise TDOA (all pairs) and config 2
estLocC2 = loc(rphatC2, sm2, 'tdoa2');

%%

figure(4)
plot(estLocC2(1,:), estLocC2(2,:))
title('Estimated location for object; TDOA2 with all pairs used')


