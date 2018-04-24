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
% TODO: Make this tdoa1?
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
figure(1)
sm1.plot
xlim([0 1.23])
ylim([0 1])

%figure(2)
sm2.plot
xlim([0 1.23])
ylim([0 1])
%% Analysis of model

% TODO: Code below redudant?
% Creating SIG object from measurments
sigrphat = sig(rphat,sampRate);
 
% CRLB analysis. Uncomment for plot
% TODO: Can we use another approach than TDOA2? (Since Hendeby said we had to use TDOA1 somewhere)
figure(1)
hold on
%Empty y, limits set according to lab setup and calc of RMSE
crlb2(sm1, [], 0:0.1:1.3, 0:0.1:1, [1 2], 'rmse'); 
title('CRLB analysis of config 1')

figure(2)
hold on
%Empty y, limits set according to lab setup and calc of RMSE
crlb2(sm2, [], 0:0.1:1.3, 0:0.1:1, [1 2], 'rmse');  
title('CRLB analysis of config 2')

hold off



%% Localization
% Config 2 is choosen as preferred config (due to config analysis)

% TODO: Make this into a function

% Load config 2
load('testconfig2.mat')

% Calculating range [meter] from time [second].
rphatC2 = 343 * tphat;

% Trajection estimation with pairwise TDOA (all pairs) and config 2
%estTrajTDOA2 = loc(rphatC2, sm2, 'tdoa2');

% Plotting trajectory
%figure(4)
%plot(estTrajTDOA2(1,:), estTrajTDOA2(2,:))
%title('Estimated location for object; TDOA2 with all pairs used')

% Comparing to ground truth
%figure(5)
%SFlabCompEstimGroundTruth(estTrajC2,micPos2)

%% TODO: This loc alg. I have no idea how to use r0 and why it gets so big
% Trajection estimation with NLS and Guass-Newton search
sm2nls = exsensor('tdoa1',8,1,2);
sm2nls.x0 = [startPos' 0.5]';
sm2nls.th = micPos2(:);
sm2nls.pe = diag(var(e));

estTrajNls3d = loc(rphatC2, sm2nls, 'nlsGn');
%%
% Plotting trajectory
figure(7)

plot(estTrajNls3d(1,:),estTrajNls3d(2,:))
title('Estimated location for object; NLS with GN search used')


%% Choosing motion models

% cv2d - Cartesian velocity in 2d
m1 = exlti('cv2d', 0.5)
%mm1=addsensor(m1,sm2)

%Artificial measurements from NLS GN Loc. alg.
artMeasVec = estTrajNls3d(1:2, :);

artMeas = sig(estTrajNls3d(1:2, :)');

% x and y for start position and 0 velocity (both directions)
%m1.x0 = [startPos; 0; 0];
% TODO: Change P0?
%m1.px0 = 0.01*eye(4);
% TODO: Change pe
%m1.pe = [1;1;1;1]';
% TODO: Change
%m1.pv =000.1* eye(2);

% Tracking using KF
xhat11 = kalman(m1, artMeas);

% Plotting result
figure(8)
xplot2(xhat11)


% ctpvd - Coordinated turn, with polar velocity


