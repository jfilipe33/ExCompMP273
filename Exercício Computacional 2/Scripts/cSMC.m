%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Controle de Posição Adaptativo                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef cSMC

    properties
        
        % Sinais
        
        x1      % Erro de Posição
        x2      % Erro de Velocidade
        a_      % Comando de Aceleração
        u       % Comando de Controle
        sig     % Variável de Deslizamento
        kappa   % Ganho de Chaveamento
        
        % Parâmetros Fixos
        
        m       % Massa do MAV
        C       % Matriz de Constantes multiplicadoras em sigma
        g       % Aceleração da Gravidade
        e3      % Vetor unitário eixo z
        Ts      % Tempo de Amostragem
        gamma   % Sintonia da taxa de variação de Kappa
        eps     % Sintonia de região de "Steady State"
        
    end
    
    methods
        
        % Construtor de Objetos
        
        function obj = cSMC( sSMC )
            
            % Inicialização de Parâmetros
            
            obj.kappa = sSMC.kappa;
            obj.C = sSMC.C;
            obj.m = sSMC.m;         
            obj.g = sSMC.g;
            obj.Ts = sSMC.Ts;
            obj.gamma = sSMC.gamma;
            obj.eps = sSMC.eps;
            
            obj.e3 = [0;0;1];
            obj.u = zeros(3,1);
            
        end
        
        % Comando de Controle
        
        function obj = ctr( obj )
            
            obj.sig = obj.C*obj.x1 + obj.x2;
            if norm(obj.sig) < obj.eps
                       ind = 0;
                    else
                       ind = 1;
                    end
            kappad = obj.gamma*norm(obj.sig)*ind;
            obj.kappa = obj.kappa + obj.Ts*kappad;
            obj.u = -obj.m*(obj.C*obj.x2 - obj.g*obj.e3 - obj.a_ + inv(obj.m)*obj.kappa*(obj.sig/norm(obj.sig)));
            
        end
        
    end
end