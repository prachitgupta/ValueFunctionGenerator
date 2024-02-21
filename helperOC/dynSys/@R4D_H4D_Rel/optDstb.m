function dOpt = optDstb(obj, ~, ~, deriv, dMode, ~)
    % uOpt = optCtrl(obj, t, y, deriv, uMode, dims)
    
    %% Input processing
    if nargin < 5
      dMode = 'min';
    end
    
    convert_back = false;
    if ~iscell(deriv)
      convert_back = true;
      deriv = num2cell(deriv);
    end
    
    dims = obj.dims;
    dOpt = cell(obj.nd, 1);
    %% Optimal disturbance
    if strcmp(dMode, 'max')
      dOpt{1} = (deriv{5} >= 0)*(obj.dMax(1)) + ...
                ((deriv{5})<0)*(obj.dMin(1));
      
      dOpt{2} = ((deriv{3})>=0)*(obj.dMax(2)) + ...
                ((deriv{3})<0)*(obj.dMin(2));

    elseif strcmp(dMode, 'min')

      dOpt{1} = (deriv{5} >= 0)*(obj.dMin(1)) + ...
                ((deriv{5})<0)*(obj.dMax(1));

      dOpt{2} = ((deriv{3})>=0)*(obj.dMin(2)) + ...
        ((deriv{3})<0)*(obj.dMax(2));
    else
        error('Unknown dMode!')
    end
    
    if convert_back
        dOpt = cell2mat(dOpt);
    end
    end
