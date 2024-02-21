function dx = dynamics(obj, ~, x, u, d, ~)
    % dx = dynamics(obj, ~, x, u, ~)
    %     Relative Dynamics of the human robot interaction modeled as zero sum differential game
    %         B = atan((lr/(lr+lf))*tan(u(2)))
    %         \dot x_1 = x_4(x2)sin(B)/lr + x5cos(x3) - x4cos(B)
    %         \dot x_2 =  -x_4(x1)sin(B)/lr + x5sin(x3) - x4sin(B)
    %         \dot x_3 = d(2) - x4sin(B)/lr
    %         \dot x_4 =  u(1)
    %         \dot x_5 = d(2)
    
    
    
    if nargin < 5
      d = {0; 0};
      
    end
    
    if nargin < 6
      dims = obj.dims;
    end
    
    convert2num = false;
    if ~iscell(x)
      x = num2cell(x);
      convert2num = true;
    end
    
    if ~iscell(u)
      u = num2cell(u);
    end
    
    if ~iscell(d)
      d = num2cell(d);
    end
    
    dx = cell(length(dims), 1);
    
    
    %% calling function with switch and filling 5x1 cell i.e synamics matrix
    for i = 1:length(dims)
      %display(d);
      dx{i} = dynamics_cell_helper(obj, x, u, d, dims, dims(i));
    end
    
    if convert2num
      dx = cell2mat(dx);
    end
    
    end
    
    function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)
    %% car lengths data from jason kong paper in m
    lr = 1.738;
    lf = 1.105;
    B = atan((lr/(lr+lf))*tan(u{2}));
    switch dim
        case 1
            dx = x{dims==4}.*x{dims==2}.*sin(B)/lr + x{dims==5}.*cos(x{dims==3}) - x{dims==4}.*cos(B) ;
        case 2
            dx = -x{dims==4}.*x{dims==1}.*sin(B)/lr + x{dims==5}.*sin(x{dims==3}) - x{dims==4}.*sin(B) ;
        case 3
            dx = d{2} - x{dims==4}.*sin(B)/lr;
        case 4
            dx = u{1};
        case 5
            dx = d{1};

      otherwise
        error('Only dimension 1-6 are defined for dynamics of Q6D_Q3D_Rel!')
    end
    end
    
