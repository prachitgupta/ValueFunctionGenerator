classdef R4D_H4D_Rel < DynSys
    properties
      uMin        % Control bounds (2x1 vector)
      uMax
      
      dMin
      dMax
              
      % active dimensions
      dims
    end
    
    methods
      function obj = R4D_H4D_Rel(x, uMin, uMax, dMin, dMax, dims)
          % obj = (x, uMin, uMax, dMin, dMax,  dims)
          % Relative dynamics between robot car using standard kinematic bicycle model and simple 4D human car model 
          if ~iscolumn(x)
            x = x';
          end
          
          if nargin < 1 || isempty(x)
              x = zeros(obj.nx, 1);
          end
          
          %%process control
          if nargin < 2
              RaccMax = 3;
              RcurvatureMax = 0.2;
              uMin = [-RaccMax; -RcurvatureMax];
              uMax = [RaccMax; RcurvatureMax];
          end
          
          if nargin < 4
            HaccMax = 3;
            HomegaMax = 0.2;
            dMin = [-HaccMax; -HomegaMax];
            dMax = [HaccMax; HomegaMax];
          end
          
          if nargin < 6
              dims = 1:5;
          end
          
          obj.x = x;
          obj.xhist = x;
          
          obj.uMax = uMax;
          obj.uMin = uMin;
          obj.dMax = dMax;
          obj.dMin = dMin;
                   
          obj.dims = dims;
          obj.nx = length(dims);
          obj.nu = 2;
          obj.nd = 2;
      end
    end
  end 
