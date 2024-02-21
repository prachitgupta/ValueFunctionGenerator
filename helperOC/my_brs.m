function [data, tau, extraOuts] = my_brs(grid_size)
    % 1. Run Backward Reachable Set (BRS) with a goal
    %     uMode = 'min' <-- goal
    %     minWith = 'none' <-- Set (not tube)
    %     compTraj = false <-- no trajectory
    % 2. Run BRS with goal, then optimal trajectory
    %     uMode = 'min' <-- goal
    %     minWith = 'none' <-- Set (not tube)
    %     compTraj = true <-- compute optimal trajectory
    % 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
    %     uMode = 'min' <-- goal
    %     minWith = 'minVOverTime' <-- Tube (not set)
    %     compTraj = true <-- compute optimal trajectory
    % 4. Add disturbance
    %     dStep1: define a dMax (dMax = [.25, .25, 0];)
    %     dStep2: define a dMode (opposite of uMode)
    %     dStep3: input dMax when creating your DubinsCar
    %     dStep4: add dMode to  schemeData
    % 5. Change to an avoid BRT rather than a goal BRT
    %     uMode = 'max' <-- avoid
    %     dMode = 'min' <-- opposite of uMode
    %     minWith = 'minVOverTime' <-- Tube (not set)
    %     compTraj = false <-- no trajectory
    % 6. Change to a Forward Reachable Tube (FRT)
    %     add schemeData.tMode = 'forward'
    %     note: now having uMode = 'max' essentially says "see how far I can
    %     reach"
    % 7. Add obstacles
    %     add the following code:
    %     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
    %     HJIextraArgs.obstacles = obstacles;
    % 8. Add random disturbance (white noise)
    %     add the following code:
    %     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
    
    
    %% Should we compute the trajectory?
    compTraj = false;
    
    %% Grid
    grid_min = [-15; -5; 0; 1; 1]; % Lower corner of computation domain
    grid_max = [15; 5; 2*pi ;12 ;12];    % Upper corner of computation domain
    N = grid_size;         % Number of grid points per dimension
    pdDims = 3;               % 3rd dimension is periodic
    g = createGrid(grid_min, grid_max, N, pdDims);
    % Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
    % state space dimensions
    
    %% target set
    R = 1;
    % data0 = shapeCylinder(grid,ignoreDims,center,radius)
    data0 = shapeCylinder(g,3:5,[0,0,0,0,0],R);
    % also try shapeRectangleByCorners, shapeSphere, etc.
    
    %% time vector
    t0 = 0;
    tMax = 2;
    dt = 0.05;
    tau = t0:dt:tMax;
    
    %% problem parameters
    
    % input bounds
    aMax= 3;
    curvatureMax = 0.2;
    omegaMax  = 0.2;

    uMin = [-aMax ; -curvatureMax];
    uMax = [aMax ; curvatureMax];

    dMin = [-aMax ; -omegaMax];
    dMax = [aMax ; omegaMax];

    % do dStep1 here
    
    % control trying to min or max value function?
    uMode = 'max';
    dMode = 'min';
    % do dStep2 here
    
    %% Pack problem parameters
    
    % Define dynamic system
    % obj = DubinsCar(x, wMax, speed, dMax)
    KBMCar = R4D_H4D_Rel([0, 0, 0, 0, 0, 0], uMin, uMax, dMin, dMax,1:5); %do dStep3 here
    
    % Put grid and dynamic systems into schemeData
    schemeData.grid = g;
    schemeData.dynSys = KBMCar;
    schemeData.accuracy = 'low'; %set accuracy
    schemeData.uMode = uMode;
    schemeData.dMode = dMode;
    schemeData.tMode = 'backward';
    %do dStep4 here
    
    %% additive random noise
    %do Step8 here
    %HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
    % Try other noise coefficients, like:
    %    [0.2; 0; 0]; % Noise on X state
    %    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
    %    [0.2;0.2;0.5]; % Coupled noise on all states
    %    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise
    
    %% If you have obstacles, compute them here
    
    %% Compute value function
    
    %HJIextraArgs.visualize = true; %show plot
    %HJIextraArgs.visualize.valueSet = 0;
    %HJIextraArgs.visualize.initialValueSet = 1;
    %HJIextraArgs.visualize.figNum = 1; %set figure number
    %HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
    
    % uncomment if you want to see a 2D slice
    HJIextraArgs.visualize.plotData.plotDims = [1 1 0 0 0]; %plot x, y
    HJIextraArgs.visualize.plotData.projpt = [0,0,0]; %project at theta = 0
    HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D
    % set target function.
    HJIextraArgs.targetFunction = data0;
    compMethod = 'minVWithL';
    [data, tau, extraOuts] = ...
    HJIPDE_solve(data0, tau, schemeData,compMethod, HJIextraArgs);
    
    %[data, tau2, ~] = ...
     % HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);
    
    %% Compute optimal trajectory from some initial state
    if compTraj
      
      %set the initial state
      xinit = [2, 2, 0,2,2];
      
      %check if this initial state is in the BRS/BRT
      %value = eval_u(g, data, x)
      value = eval_u(g,data(:,:,:,:,:,end),xinit);
      
      if value <= 0 %if initial state is in BRS/BRT
        % find optimal trajectory
        
        dCar.x = xinit; %set initial state of the dubins car
    
        TrajextraArgs.uMode = uMode; %set if control wants to min or max
        TrajextraArgs.dMode = 'min';
        TrajextraArgs.visualize = true; %show plot
        TrajextraArgs.fig_num = 2; %figure number
        
        %we want to see the first two dimensions (x and y)
        TrajextraArgs.projDim = [1 1 0]; 
        
        %flip data time points so we start from the beginning of time
        dataTraj = flip(data,4);
        
        % [traj, traj_tau] = ...
        % computeOptTraj(g, data, tau, dynSys, extraArgs)
        [traj, traj_tau] = ...
          computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
      
        figure(6)
        clf
        h = visSetIm(g, data(:,:,:,end));
        h.FaceAlpha = .3;
        hold on
        s = scatter3(xinit(1), xinit(2), xinit(3));
        s.SizeData = 70;
        title('The reachable set at the end and x_init')
        hold off
      
        %plot traj
        figure(4)
        plot(traj(1,:), traj(2,:))
        hold on
        xlim([-5 5])
        ylim([-5 5])
        % add the target set to that
        [g2D, data2D] = proj(g, data0, [0 0 1]);
        visSetIm(g2D, data2D, 'green');
        title('2D projection of the trajectory & target set')
        hold off
      else
        error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
      end
    end
    end
