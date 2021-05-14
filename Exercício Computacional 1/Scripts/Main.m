%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Multi-Input SMC - Main Script                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;

%% Parâmetros de Simulação

tf = 16;               % Duração da Simulação [s]
Ts  = 0.001;           % Período de Amostragem

%% Objetos

% MAV

sMAV.r = [0;0;0];    % Posição Inicial
sMAV.v = [0;0;0];    % Velocidade Inicial
sMAV.t = 0;          % Tempo Inicial
sMAV.f = 0.25;       % Frequência do Distúrbio
sMAV.Ts = Ts;        % Período de Amostragem
sMAV.m = 1;          % Massa do MAV
sMAV.g = 9.81;       % Aceleração da Gravidade

oMAV = cMAV( sMAV );

% Controlador por Modos Deslizantes

sSMC.kappa = 1;
sSMC.C = 1*eye(3);
sSMC.m = sMAV.m;
sSMC.g = sMAV.g;

oSMC = cSMC( sSMC );

% Gerador de Comando de Posição

sComando.t = sMAV.t;
sComando.Ts = Ts;
oComando = cComando( sComando );

% Simulação

for k = 1:tf/Ts
    
    %Gerando comandos de Pos/Vel/Acc
    oComando = destino( oComando );
    
    %Gerando comando de controle
    oSMC.x1 = oMAV.r - oComando.r_;
    oSMC.x2 = oMAV.v - oComando.v_;
    oSMC.a_ = oComando.a_;
    
    oSMC = ctr( oSMC );
    
    %Simulação da planta
    oMAV.a_ = oComando.a_;
    oMAV.u = oSMC.u;
    
    oMAV = dist( oMAV );
    oMAV = prop( oMAV );
    
    oComando.t = oMAV.t;
    
    %Dados para plotar
    dest(:,k) = oComando.r_;
    path(:,k) = oMAV.r;
    sigma(:,k) = oSMC.sig;
    u(:,k) = oSMC.u;
    comv(:,k) = oComando.v_;
    vel(:,k) = oMAV.v;
    d(:,k) = oMAV.d;  
    
end

Tc = norm(sigma(1,:))/(oSMC.kappa - 0.5831) %Reaching Time Upper Limit

figure; hold on; grid; box;
title('MAV Position over time');
plot3(dest(1,:),dest(2,:),dest(3,:),'LineWidth',2,'LineStyle','--');
plot3(path(1,:),path(2,:),path(3,:),'LineWidth',2);
legend('$\bar{r}$','$r$','interpreter','latex','FontSize',20);

figure; hold on; grid; box;
title('MAV Velocity over time');
plot3(comv(1,:),comv(2,:),comv(3,:),'LineWidth',3,'LineStyle','--');
plot3(vel(1,:),vel(2,:),vel(3,:),'LineWidth',2);
legend('$\bar{v}$','$v$','interpreter','latex','FontSize',20);


figure
subplot(3,1,1)
plot(0:Ts:tf-Ts,sigma(1,:))
title('Sliding Variables');
ylabel('$\sigma_1$','interpreter','latex','FontSize',14);
subplot(3,1,2)
plot(0:Ts:tf-Ts,sigma(2,:))
ylabel('$\sigma_2$','interpreter','latex','FontSize',14);
subplot(3,1,3)
plot(0:Ts:tf-Ts,sigma(3,:))
ylabel('$\sigma_3$','interpreter','latex','FontSize',14);
xlabel('Tempo (s)','interpreter','latex','FontSize',14);

figure('Name','Sliding Variables'); grid on; hold;
plot(0:Ts:tf-Ts,sigma(1,:),'LineWidth',2);
plot(0:Ts:tf-Ts,sigma(2,:),'LineWidth',2);
plot(0:Ts:tf-Ts,sigma(3,:),'LineWidth',2);
title('Sliding Variables')
xlabel('Time','FontSize',14)
ylabel('Amplitude','FontSize',14)
legend('$\sigma_1$','$\sigma_2$','$\sigma_3$','interpreter','latex','FontSize',20);

figure
subplot(3,1,1)
plot(0:Ts:tf-Ts,u(1,:))
title('Switching Term');
ylabel('$\nu_1$','interpreter','latex','FontSize',14);
subplot(3,1,2)
plot(0:Ts:tf-Ts,u(2,:))
ylabel('$\nu_2$','interpreter','latex','FontSize',14);
subplot(3,1,3)
plot(0:Ts:tf-Ts,u(3,:))
ylabel('$\nu_3$','interpreter','latex','FontSize',14);
xlabel('Tempo (s)','interpreter','latex','FontSize',14);

figure('Name','Switching Term'); grid on; hold;
plot(0:Ts:tf-Ts,u(1,:),'LineWidth',2);
plot(0:Ts:tf-Ts,u(2,:),'LineWidth',2);
plot(0:Ts:tf-Ts,u(3,:),'LineWidth',2);
title('Switching Term')
xlabel('Time','FontSize',14)
ylabel('u','FontSize',14)
legend('$\mathbf{u}_1$','$\mathbf{u}_2$','$\mathbf{u}_3$','interpreter','latex','FontSize',20);

figure('Name','Disturbance Phase Portrait'); hold on; grid; box;
title('Disturbance Phase Portrait');
plot3(d(1,:),d(2,:),d(3,:),'LineWidth',2);
scatter3(0,0,0,'LineWidth',3);
xlabel('$d_x$','interpreter','latex','FontSize',14);
ylabel('$d_y$','interpreter','latex','FontSize',14);
zlabel('$d_z$','interpreter','latex','FontSize',14);

