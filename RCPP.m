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

function [optimal_path, min_cost] = RCPP(M, dx, starting_point, ending_point, translation_speed, rot_spd)
% Rotating calipers path planner 
% 
% M : Polygon
% dx distance between flight lines
% translation_speed and rot_spd are used to estimate the cost of a path.
% The speed affects the behaviour of the planner.

A = antipodalPoints(M);
[m, ~] = size(A);

x_start = starting_point(1);
y_start = starting_point(2);
x_end = ending_point(1);
y_end = ending_point(2);

min_cost = Inf;
optimal_path = [];
best_antipodal_pair = 0;

for i=1:m 
    Path = bestPathForAntipodalPair(M, A(i,:), dx);
         
    %check if the path should be inverted
    FullPath1 = [x_start y_start; Path;  x_end y_end];
    FullPath2 = [x_start y_start; flipud(Path); x_end y_end];
    
    cost1 = timeCost2D(FullPath1, translation_speed, rot_spd, [x_start y_start 0]);
    
    cost2 =  timeCost2D(FullPath2, translation_speed, rot_spd, [x_start y_start 0]);
    
    if (cost1 < cost2)
       FullPath = FullPath1;
       Cost(i) = cost1;
    else
       FullPath = FullPath2;
       Cost(i) = cost2;
    end
    
    if Cost(i)<min_cost
        min_cost = Cost(i);
        optimal_path = FullPath;
        best_antipodal_pair = i;
    end
end

end

