%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MP-273: Sliding mode control                                            %
% Author: João Filipe Silva                                               %
% Affiliation: Aeronautics Institute of Technology (ITA/Brazil)           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: EXAME - Observer Class Script                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef cOBS
    
    properties
        
        %Signals
        
        yhat           %Mass Initial Position
        ydhat          %Mass Initial Velocity
        sig            %Sliding Variable
        
        %Fixed Properties
        
        m           %Block Mass
        b           %Friction Coefficient
        k0          %Spring Constant
        k1          %Spring Constant
        Ts          %Sampling Time
        a1          %yhat Linear Gain 1 
        a2          %yhat Linear Gain 2
        kappa1      %yhat NonLinear Gain 1
        kappa2      %yhat NonLinear Gain 2
        kst1        %Super-Twisting Gain 1
        kst2        %Super-Twisting Gain 2
        rho         %Disturbance Upper BOund
       
    end
    
    methods
        function obj = cOBS( sOBS )
            
            obj.m = sOBS.m;
            obj.b = sOBS.b;
            obj.k0 = sOBS.k0;
            obj.k1 = sOBS.k1;
            obj.yhat = 0;
            obj.ydhat = 0;
            obj.Ts = sOBS.Ts;
            obj.rho = sOBS.rho;
            
            obj.a1 = 1;
            obj.a2 = 1;
            obj.kappa1 = 0.3;
            obj.kappa2 = 1;
            obj.kst1 = 1.5*sqrt(obj.rho);
            obj.kst2 = 1.1*obj.rho;
            
        end       
        
        function obj = slotine( obj )
            
            obj.yhat = obj.yhat + obj.Ts*(obj.a1*obj.sig + obj.ydhat + obj.kappa1*sign(obj.sig));
            ydd = inv(obj.m)*(-obj.k1*obj.yhat^3 - obj.k0*obj.yhat - obj.b*obj.ydhat*abs(obj.ydhat));
            obj.ydhat = obj.ydhat + obj.Ts*(obj.a2*obj.sig + ydd + obj.kappa2*sign(obj.sig));            
            
        end
        
        function obj = stsmo( obj )
            
            obj.yhat = obj.yhat + obj.Ts*(obj.ydhat + obj.kst1*sqrt(abs(obj.sig))*sign(obj.sig));
            ydd = inv(obj.m)*(-obj.k1*obj.yhat^3 - obj.k0*obj.yhat - obj.b*obj.ydhat*abs(obj.ydhat));
            obj.ydhat = obj.ydhat + obj.Ts*(ydd + obj.kst2*sign(obj.sig)); 
            
        end
    end
end

