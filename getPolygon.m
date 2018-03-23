% CONACYT
% Copyright (c) 2017, J. Irving Vasquez-Gomez,
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the <organization> nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL J. Irving Vasquez-Gomez BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



function [Polygon_vertex, shifted_polygon_vertex] = getPolygon(numVert, radius, radVar, angVar)
% Generates a convex polygon

% Specify polygon variables
%numVert = 8;
%radius = 1;
%radVar = 1; % variance in the spikiness of vertices
%angVar = 1; % variance in spacing of vertices around the unit circle

% preallocate x and y 
x = zeros(numVert,1);
y = zeros(numVert,1);

% angle of the unit circle in radians
circleAng = 2*pi;
% the average angular separation between points in a unit circle
angleSeparation = circleAng/double(numVert);
% create the matrix of angles for equal separation of points
angleMatrix = 0: angleSeparation: circleAng;
% drop the final angle since 2Pi = 0
angleMatrix(end) = [];

% generate the points x and y
for k = 1:numVert
    x(k) = (radius + radius*rand(1)*radVar) * cos(angleMatrix(k) + angleSeparation*rand(1)*angVar);
    y(k) = (radius + radius*rand(1)*radVar) * sin(angleMatrix(k) + angleSeparation*rand(1)*angVar);
end

Polygon_vertex = [x y];

%reverse to clock wise
Polygon_vertex = flipud(Polygon_vertex);

shifted_polygon_vertex = circshift(Polygon_vertex, -1);
%M = [Polygon_vertex shifted_polygon_vertex];

end