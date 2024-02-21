function uOpt = optCtrl(obj, ~, x, deriv, uMode)
    % uOpt = optCtrl(obj, t, y, deriv, uMode, dims)
    
    %% Input processing
    if nargin < 5
      uMode = 'min';
    end
    
    if nargin < 6
      dims = obj.dims;
    end
    
    if ~iscell(deriv)
      deriv = num2cell(deriv);
    end
    
    uOpt = cell(obj.nu, 1);
    
    %% Optimal control
    lr = 1.738;
    lf = 1.105;
    Bmax = atan((lr/(lr+lf))*tan(obj.uMax(2)));
    Bmin = atan((lr/(lr+lf))*tan(obj.uMin(2)));
    
    det1 = deriv{4};

    SumUU = deriv{1}.*x{4}.*(x{2}).*sin(Bmax)/lr -deriv{1}.*x{4}.*cos(Bmax) ...
      - deriv{2}.*x{4}.*(x{1}).*sin(Bmax)/lr -deriv{2}.*x{4}.*sin(Bmax) ...
      - x{3}.*sin(Bmax)/lr ;

    SumUL = deriv{1}.*x{4}.*(x{2}).*sin(Bmin)/lr -deriv{1}.*x{4}.*cos(Bmin) ...
          - deriv{2}.*x{4}.*(x{1}).*sin(Bmin)/lr -deriv{2}.*x{4}.*sin(Bmin) ...
          - x{4}.*sin(Bmin)/lr ;

    if strcmp(uMode, 'max')
      uOpt{1} = ( det1 >=0)*obj.uMax(1)  + (det1 < 0)*obj.uMin(1);

      if SumUU > SumUL 
        uOpt{2} = obj.uMax(2);
      else
        uOpt{2} = obj.uMin(2);
      end
         
    elseif strcmp(uMode, 'min')
      uOpt{1} = ( det1 >=0)*obj.uMin(1)  + (det1 < 0)*obj.uMax(1);

      if SumUU > SumUL 
        uOpt{2} = obj.uMin(2);
      else
        uOpt{2} = obj.uMax(2);
      end

    else
      error('Unknown uMode!')
    end
    
    end
