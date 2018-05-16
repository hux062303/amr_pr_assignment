function [x,s]=pop(s)
% stack/pop: pushes an element onto the top of a stack
% usage: [x,s]=pop(s)
%
% s should be a stack created with push/pop or a cell array of the form
% {x,s1} where s1 is a stack. If s is empty, pop gives a warning and
% returns x=[]

% Author: Ben Petschel 28/8/2009
% Version history:
%   28/8/2009 - first release

if isempty(s)
  warning('stack:pop:empty','popped off an empty stack');
  x=[];
else
  x=s{1};
  s=s{2};
end

end