% Copyright (c) <year>, <copyright holder>
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
% DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


% Torres et al 16
% flight lines rotattion angle
% V: vertices of the polygon
% dx: distance between lines

function [Path_clock, Path_counter_clock, flra] = torres16(V, dx)
    [n,~] = size(V);
    optimal_dist = Inf;
    
    %for all edges in the polygon
    for i=1:n;
        max_dist_edge = 0;
        %for all vertex in the polygon
        for j=1:n;
            if distPoint2Line(V(i,:),V(increment(i,n),:),V(j,:))>max_dist_edge
                max_dist_edge = distPoint2Line(V(i,:),V(increment(i,n),:),V(j,:));
                opposed_vertex = j;
            end
        end
        
        if max_dist_edge < optimal_dist
            optimal_dist = max_dist_edge;
            angle = slopeAngle(V(i,:), V(increment(i,n),:));
        end
    end
    
    flra = angle;
    alpha = angle-pi/2; % R rotation is with respect of the east; alpha rotation is with respect of the north
    Pstart = V;
    Pend = circshift(V, -1);
    PsR = rotatePolygon(Pstart, -alpha);
    PsR = PsR';
    PeR = rotatePolygon(Pend, -alpha);
    PeR = PeR';

    % clock wise path
    [PathRotated] = getPathMR([PsR PeR], dx, 1);
    Path_clock = rotatePolygon(PathRotated, alpha);
    Path_clock = Path_clock';
    
    % counter clock wise path
    [PathRotated] = getPathMR([PsR PeR], dx, -1);
    Path_counter_clock = rotatePolygon(PathRotated, alpha);
    Path_counter_clock = Path_counter_clock';
end

function i_next = increment(i,n)
    i_next = mod(i,n)+1;
end

function alpha = slopeAngle(p1, p2)
    alpha = atan2(p2(2)-p1(2),p2(1)-p1(1));
end