%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: EXAME - Main Script                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean Slate
close all
clear all
clc

%% Simulation Parameters

tf = 10;                %Simulation Time
Ts = 0.0001;            %Sampling Time
t = 0;                  %Current Time Instant

%% MMA Class

sMMA.m = 1;             %Block mass
sMMA.b = 0.1;           %Friction Coefficient
sMMA.k0 = 0.05;         %Spring Constant
sMMA.k1 = 0.05;         %Spring Constant
sMMA.y = 0.1;           %Mass Initial Position
sMMA.yd = -0.1;         %Mass Initial Velocity
sMMA.rho = 0.5;         %Disturbance Upper Bound
sMMA.t = t;
sMMA.Ts = Ts;

oMMA = cMMA( sMMA );

%% Observer Class

sOBS = sMMA;
oOBS = cOBS( sOBS );

%% Simulation

for cont = 1:tf/Ts
    
    oMMA = dist( oMMA );
    oMMA = prop( oMMA );
    
    oOBS.sig = oMMA.y - oOBS.yhat;
%     oOBS = slotine( oOBS );
    oOBS = stsmo( oOBS );
    
    dp(cont) = oMMA.d;
    yp(cont) = oMMA.y;
    ydp(cont) = oMMA.yd;
    ytilde(cont) = oMMA.y - oOBS.yhat;
    yhatp(cont) = oOBS.yhat;
    ydhatp(cont) = oOBS.ydhat;
    ydtilde(cont) = oMMA.yd - oOBS.ydhat;
    
end

figure('Name','Disturbance Input'); grid on; hold;
plot(0:Ts:4-Ts,dp(1:4/Ts),'LineWidth',2);
title('Disturbance Input')
xlabel('Time [s]','FontSize',14,'interpreter','latex')
ylabel('$d$','FontSize',14,'interpreter','latex')

figure('Name','Block Position'); grid on; hold;
plot(0:Ts:tf-Ts,yp,'LineWidth',1,'LineStyle','--');
plot(0:Ts:tf-Ts,yhatp,'LineWidth',2);
title('Block Position')
xlabel('Time [s]','FontSize',14,'interpreter','latex')
ylabel('$y$','FontSize',14,'interpreter','latex')
legend('$y$','$\hat{y}$','interpreter','latex','FontSize',20);

figure('Name','Block Velocity'); grid on; hold;
plot(0:Ts:tf-Ts,ydp,'LineWidth',1,'LineStyle','--');
plot(0:Ts:tf-Ts,ydhatp,'LineWidth',2);
title('Block Velocity')
xlabel('Time [s]','FontSize',14,'interpreter','latex')
ylabel('$\dot{y}$','FontSize',14,'interpreter','latex')
legend('$\dot{y}$','$\dot{\hat{y}}$','interpreter','latex','FontSize',20);

figure('Name','Estimation Errors'); grid on; hold;
plot(0:Ts:tf-Ts,ytilde,'LineWidth',2);
plot(0:Ts:tf-Ts,ydtilde,'LineWidth',2);
title('Estimation Errors')
xlabel('Time [s]','FontSize',14,'interpreter','latex')
ylabel('$\left(\tilde{y},\dot{\tilde{y}}\right)$','FontSize',14,'interpreter','latex')
legend('$\tilde{y}$','$\dot{\tilde{y}}$','interpreter','latex','FontSize',20);

