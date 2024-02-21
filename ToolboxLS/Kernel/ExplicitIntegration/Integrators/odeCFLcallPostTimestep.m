function [ yOut, schemeDataOut ] = ...
                         odeCFLcallPostTimestep(t, yIn, schemeDataIn, options);
% odeCFLcallPostTimestep: call any postTimestep routines.
%
% [ yOut, schemeDataOut ] = ...
%                        odeCFLcallPostTimestep(t, yIn, schemeDataIn, options);
%
%
% Calls one or more postTimestep routines, depending on the contents
%   of the options.postTimestep field.  Helper routine for odeCFLn.
%
% If options.postTimestep is a cell vector of function handles, the
%   function are called in order.
%
% parameters:
%   t              Current time.
%   yIn            Input version of the level set function, in vector form.
%   schemeDataIn   Input version of a structure.
%   options        An option structure generated by odeCFLset 
%                    (use [] as a placeholder if necessary).
%
%   yOut           Output version of the level set function, in vector form.
%   schemeDataOut  Output version of the structure.
%
% The postTimestep routines called will determine whether and how
%   yOut and schemeDataOut differ from their input versions.

% Copyright 2004 Ian M. Mitchell (mitchell@cs.ubc.ca).
% This software is used, copied and distributed under the licensing 
%   agreement contained in the file LICENSE in the top directory of 
%   the distribution.
%
% Factored from odeCFLn, Ian Mitchell, 12/06/04.

  % Copy over the current version of data and scheme structure.
  yOut = yIn;
  schemeDataOut = schemeDataIn;

  % Check to see if there is anything to do.
  if(isempty(options) || isempty(options.postTimestep))
    return;
  end

  % Make the necessary calls.
  if(isa(options.postTimestep, 'function_handle'))
    [ yOut schemeDataOut ] = ...
                        feval(options.postTimestep, t, yOut, schemeDataOut);
  elseif(isa(options.postTimestep, 'cell'))
    for i = 1 : length(options.postTimestep)
      [ yOut schemeDataOut ] = ...
                        feval(options.postTimestep{i}, t, yOut, schemeDataOut);
    end
  end
