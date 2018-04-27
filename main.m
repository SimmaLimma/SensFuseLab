%% Lab in sensor fusion
% By Simon Mårtensson and Axel Nyström

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

e = zeros(size(rphat));
% e is measurement error for position estimation
for mic = 1:8
    e(:,mic) = rphat(:,mic) - meanrphat;
end

rphatPair = zeros(88,28);

rowInd = 1;
for i = 1:7
    for j = i+1:8
        rphatPair(:,rowInd) = rphat(:,i) - rphat(:,j);
        rowInd = rowInd +1;
    end
end

% Calculating e(i,j) = e(i) - e(j)
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

% Load config 2
load('testconfig2New.mat')

% Calculating range [meter] from time [second].
rphatC2 = 343 * tphat;

% Trajection estimation with pairwise TDOA (all pairs) and config 2
estTrajTDOA2 = loc(rphatC2, sm2, 'tdoa2');

% Plotting trajectory
figure(4)
plot(estTrajTDOA2(1,:), estTrajTDOA2(2,:))
title('Localisation; TDOA2 with all pairs used')

% Comparing to ground truth
figure(5)
SFlabCompEstimGroundTruth(estTrajTDOA2,micPos2)
title('Localisation; TDOA2 with all pairs used')

% Trajection estimation with NLS and Guass-Newton search
sm2nls = exsensor('tdoa1',8,1,2);
sm2nls.x0 = [startPos' 0.5]';
sm2nls.th = micPos2(:);
sm2nls.pe = diag(var(e));

estTrajNlsGn = loc(rphatC2, sm2nls, 'nlsGn');

% Plotting trajectory
figure(6)

plot(estTrajNlsGn(1,:),estTrajNlsGn(2,:))
title('NLS with GN search used')

% Comparing to ground truth
figure(7)
SFlabCompEstimGroundTruth(estTrajNlsGn(1:2, :),micPos2)
title('NLS with GN search used')

%% Choosing motion models

% Model with Constant velocity in 2d - 'cv2d'
mCv = exnl('cv2d', 0.5);

% Model with Coordinated turn and cartesian velocity
mCtcv = exnl('ctcv2d', 0.5);
% TODO: Add cartesian measurements instead of polar

% Artificial measurements from NLS GN Loc. alg.
artMeasVec = estTrajNlsGn(1:2, :);
artMeas = sig(estTrajNlsGn(1:2, :)');
% With Polar coordinates
[thTmp, rTmp] = cart2pol(estTrajNlsGn(1, :), estTrajNlsGn(2, :));
artMeasPol = sig([rTmp; thTmp]');

% Tuning the KF Filters for both models
% Tuning Q for CV
mCv.pv = 0.01*eye(4);
% Tuning R for CV
mCv.pe = 0.1*eye(2);

% Init parameters for model CV
mCv.px0 = 0.01*eye(4);
mCv.x0 = [startPos; 0; 0];


% Tuning Q for Ctcv
mCtcv.pv = 0.01*eye(5);
% Tuning R for Ctcv
mCtcv.pe = 0.1*eye(2);

% Init parameters for model CA
mCtcv.px0 = 0.01*eye(5);
% States are [pos_x, pos_y, vel_x, vel_y, ang_vel]
mCtcv.x0 = [startPos; 0; 0; 0];


% Tracking using KF (for both models)
xhatCvKF = ekf(mCv, artMeas);
xhatCtcvKF = ekf(mCtcv, artMeasPol);

% Plotting result
figure(8)
xplot2(xhatCvKF);
title('Tracking with KF and CV model (Artificial measurements)')

figure(9)
SFlabCompEstimGroundTruth(xhatCvKF.x(:,1:2)',micPos2)
title(' Tracking with KF and CV model (Artificial measurements)')

figure(10)
xplot2(xhatCtcvKF)
title('Tracking with EKF and CTCV model (Artificial measurements)')

figure(11)
SFlabCompEstimGroundTruth(xhatCtcvKF.x(:,1:2)',micPos2)
title('Tracking with EKF and CTCV model (Artificial measurements)')

%% Estimating using non-linear filter

% Selecting mic 8 as reference sensor,
% since it has low variance and a comfortable place in the array
refMic = 8;

% Calculating TOA differences for all pairs including mic 8
rphatPairOneRef = zeros(88,7);
for pairInd = 1:mic
    
    %Do not calcuate for pair ref mic with itself
    if (pairInd ~= refMic)
        rphatPairOneRef(:,pairInd) = rphatC2(:, refMic) - rphatC2(:, pairInd);
    end
end
rphatPairOneRefSig = sig(rphatPairOneRef);

%Creating function for model
h = char('[sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(1)).^2+(x(2,:)-th(2)).^2); sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(3)).^2+(x(2,:)-th(4)).^2); sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(5)).^2+(x(2,:)-th(6)).^2); sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(7)).^2+(x(2,:)-th(8)).^2); sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(9)).^2+(x(2,:)-th(10)).^2); sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(11)).^2+(x(2,:)-th(12)).^2); sqrt((x(1,:)-th(15)).^2+(x(2,:)-th(16)).^2) - sqrt((x(1,:)-th(13)).^2+(x(2,:)-th(14)).^2); ]');
    
% Dimension (nn = [nx nu ny nth])
nn = [4 0 7 16];

% Model for TDOA with all pairs including reference mic
smOneRef = sensormod(h, nn);

% Creating non-linear models with mics added as sensors
mCvNltmp = exmotion('cv2d', 1/2);
mCvNl = addsensor(mCvNltmp, smOneRef);

% Tuning the EKF 
% Tuning Q
mCvNl.pv = 0.005*eye(4);
% Tuning R
mCvNl.pe = diag(var(rphatPairOneRef));

% Init parameters for model CV
mCvNl.px0 = 0.01*eye(4);
mCvNl.x0 = [startPos; 0; 0];
mCvNl.th = micPos2(:);

% Tracking using EKF 
rphatPairSig = sig(rphatPair);
xhatCvEKF = ekf(mCvNl, rphatPairOneRefSig);

% Plotting result
figure(12)
xplot2(xhatCvEKF)
title('Tracking with EKF and TOA (one ref) model, using CV model')

figure(13)
SFlabCompEstimGroundTruth(xhatCvEKF.x(:,1:2)',micPos2)
title('Tracking with EKF and TOA (one ref) model, using CV model')


%%
% Creating non-linear models with mics added as sensors
mCtcvNltmp = exmotion('ctcv2d', 1/2);
mCtcvNl = addsensor(mCtcvNltmp, smOneRef);

% Tuning the EKF 
% Tuning Q
mCtcvNl.pv = 0.005*eye(5);
% Tuning R
mCtcvNl.pe = diag(var(rphatPairOneRef));

% Init parameters for model CV
mCtcvNl.px0 = 0.01*eye(5);
mCtcvNl.x0 = [startPos; 0; 0; 0];
mCtcvNl.th = micPos2(:);

% Tracking using EKF 
xhatCtcvEKF = ekf(mCtcvNl, rphatPairOneRefSig);

% Plotting result
figure(14)
xplot2(xhatCtcvEKF)
title('Tracking with EKF and TOA (one ref) model, using CTCV model')

figure(15)
SFlabCompEstimGroundTruth(xhatCtcvEKF.x(:,1:2)',micPos2)
title('Tracking with EKF and TOA (one ref) model, using CTCV model')


%% Sensitivity analysis
% Choosing method with tracking, using EKF, Pairwise TOA (one ref)
% and CV2D motion model

% Adding offsets to microphone positions
offset = 0.1;
micPos2Off = micPos2 + offset*(2*rand(2,8) - ones(2,8));

% Reusing model, but with new mic positions
mCvNl.th = micPos2Off(:);

% Estimating new trajectory
xhatCvEKFOff = ekf(mCvNl, rphatPairOneRefSig);

% Plotting result
figure(16)
xplot2(xhatCvEKFOff)
title('Comparison with random offset at mic pos')

figure(17)
SFlabCompEstimGroundTruth(xhatCvEKFOff.x(:,1:2)',micPos2Off)
title('Random offset for mics')
