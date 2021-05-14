%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: MAV - Posição                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef cMAV

    properties
        
        % Sinais
        
        r       % Posição tridimensional
        v       % Velocidade
        a_      % Comando de Aceleração
        u       % Comando de Controle
        d       % Distúrbio Incidente
        t       % Tempo
        
        % Parâmetros Fixos
        
        f       % Frequência do Distúrbio
        Ts      % Tempo de Amostragem / Período de Integração
        m       % Massa do MAV
        g       % Aceleração da Gravidade
        e3      % Vetor unitário eixo z
        
    end
    
    methods
        
        % Construtor de Objetos
        
        function obj = cMAV( sMAV )
            
            % Inicialização de Parâmetros
            
            obj.r = sMAV.r;
            obj.v = sMAV.v;
            obj.f = sMAV.f;
            obj.Ts = sMAV.Ts;
            obj.m = sMAV.m;
            obj.g = sMAV.g;
            obj.t = sMAV.t;
            
            obj.d = 0;
            obj.e3 = [0;0;1];
            
        end
        
        % Gerador de Distúrbio
        
        function obj = dist( obj )
            obj.d = [0.3 * sin(2*pi*obj.f*obj.t);
                     0.4 * sin(2*pi*obj.f*obj.t + pi/2);
                     0.5 * sin(2*pi*obj.f*obj.t + pi)];
        end
        
        
        function obj = prop( obj )
            
            obj.r = obj.r + obj.Ts*obj.v;
            %obj.a = inv(obj.m)*obj.u - obj.g*obj.e3 + inv(obj.m)*obj.d;
            obj.v = obj.v + obj.Ts*(inv(obj.m)*obj.u - obj.g*obj.e3 + inv(obj.m)*obj.d);
            
            obj.t = obj.t + obj.Ts;
        end
        
    end
end

