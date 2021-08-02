classdef tPowerLine < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        
        % Properties or Parameters
        pCAD   % Tarot 650-Sport 3D image
        pPar   % Parameters
        pID    % Identification
        
        % Control variables
        pPos   % Posture
        pFlag  % Flags
        

    end
    
    methods
        function car = tPowerLine(ID)
            if nargin < 1
                ID = 1;
            end
            car.pID = ID;
                
            iControlVariables(car);
            iParameters(car);
            mCADload(car);
      
        end
        
        % ==================================================
        iControlVariables(car);
        iParameters(car);
        
        % ==================================================
        % Tarot 3D Image
        mCADload(car);
        mCADplot(car);
        mCADcolor(car,color);

      
    end
end