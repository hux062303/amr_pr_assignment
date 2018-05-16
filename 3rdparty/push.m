function s=push(x,s)
% stack/push: pushes an element onto the top of a stack
% usage: s=push(x,s)
%
% s can be either an empty cell array (empty stack) or a stack created with
% push/pop.  A non-empty stack is a cell array {x,s1} where s1 is a stack.
%
% Warning:
% matlab (r2009a) may terminate unexpectedly if the stack size is too
% large.

% Author: Ben Petschel 28/8/2009
% Version history:
%   28/8/2009 - first release

s = {x,s};

end