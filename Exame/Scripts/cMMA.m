%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: EXAME - Mass-Spring-Damper Class Script                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef cMMA
    
    properties
        
        %Signals
        
        y           %Mass Initial Position
        yd          %Mass Initial Velocity
        t           %Current Time Instant
        d           %Disturbance
        
        %Fixed Properties
        
        m           %Block Mass
        b           %Friction Coefficient
        k0          %Spring Constant
        k1          %Spring Constant
        Ts          %Sampling Time
        rho         %Disturbance Upper Bound
       
    end
    
    methods
        function obj = cMMA( sMMA )
            
            obj.m = sMMA.m;
            obj.b = sMMA.b;
            obj.k0 = sMMA.k0;
            obj.k1 = sMMA.k1;
            obj.y = sMMA.y;
            obj.yd = sMMA.yd;
            obj.Ts = sMMA.Ts;
            obj.t = sMMA.t;
            obj.rho = sMMA.rho;
            obj.d = 0;
            
        end
        
        function obj = dist( obj )
            
             obj.d = obj.rho*(0.8*sin(2*pi*2*obj.t) + 0.2*sin(2*pi*5*obj.t));
            
        end
        
        function obj = prop( obj )
            
            obj.y = obj.y + obj.Ts*(obj.yd);
            ydd = inv(obj.m)*(obj.d - obj.k1*obj.y^3 - obj.k0*obj.y - obj.b*obj.yd*abs(obj.yd));
            obj.yd = obj.yd + obj.Ts*(ydd);
            
            obj.t = obj.t + obj.Ts;
            
        end
    end
end

