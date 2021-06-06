%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: MAV - Posição                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef cComando

    properties
        
        % Sinais
        
        r_          % Comando de Posição
        v_          % Comando de Velocidade
        a_          % Comando de Aceleração
        r_prev      % Comando de Posição anterior
        v_prev      % Comando de Velocidade anterior
        t           % Tempo
        Ts          % Período de Amostragem
        
    end
    
    methods
        
        % Construtor de Objetos
        
        function obj = cComando( sComando )
            
            % Inicialização de Parâmetros
            
            obj.r_ = zeros(3,1);
            obj.v_ = zeros(3,1);
            obj.a_ = zeros(3,1);
            obj.t = sComando.t;
            obj.Ts = sComando.Ts;
            
        end
        
        % Gerador de Destino
        
        function obj = destino( obj )
            
            obj.r_ = [sin(pi*obj.t/2);
                      sin(pi*obj.t/2 + pi/2);
                      obj.t/4];
            %v_ = d(r_)/dt
            obj.v_ = [0.5*pi*cos(pi*obj.t/2);
                      -0.5*pi*sin(pi*obj.t/2);
                      1/4];
            %a_ = d(v_)/dt
            obj.a_ = [-0.25*pi^2*sin(pi*obj.t/2);
                      -0.25*pi^2*cos(pi*obj.t/2);
                      0];

        end
        
    end
end

