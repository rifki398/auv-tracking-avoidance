function angle = ssa(angle,unit)
% SSA is the "Smallest-Signed Angle" or the smallest difference between two
% angles. 
% 
% Examples:
%   angle = ssa(angle) maps an angle in rad to the interval [-pi pi) 
%   angle = ssa(angle,'deg') maps an angle in deg to the interval [-180 180)
%
% For feedback control systems and state estimators used to control the 
% attitude of vehicles, the difference of two angles should always be
% mapped to [-pi pi) or [-180 180) to avoid step inputs/discontinuties.           
%
% Note that in many languages (C, C++, C#, JavaScript), the modulus  
% operator mod(x,y) returns a value with the same sign as x. 
% For these use a custom mod function: mod(x,y) = x - floor(x/y) * y
% For the Unity game engine use: Mathf.DeltaAngle.
%
% Author:     Thor I. Fossen
% Date:       2018-09-21
% Revisions:  
%   2020-03-04  Deafult rad, optional argument for degrees.
%   2024-12-04  Allow rad as optional argument. Throw error for invalid
%   unit. Author: Tor Børve Rasmussen.

if nargin == 1 || strcmp(unit, 'rad')
    angle = mod( angle + pi, 2 * pi ) - pi; 
elseif strcmp(unit,'deg')
    angle = mod( angle + 180, 360 ) - 180; 
else
    error("MSS:InvalidArgument", "Invalid unit argument: \'%s\'. " + ...
        "Unit must be either \'deg\' or \'rad\'", unit);
end
    
