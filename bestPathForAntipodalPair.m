% CONACYT
% Copyright (c) 2017, Juan Irving Vasquez-Gomez,
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




% Best path for antipodal pairs
% A: antipodal pairs
% V: vertices of the polygon
% 

function [Path, inclination] = bestPathForAntipodalPair(V, A, dx) 
    [n,~] = size(V);

    i = A(1,1);
    j = A(1,2);

    %if angleAP(V, i,j) < angleAP(V, j, i)
    if (angleAP(V,i,j) - pi) < 0
        b1 = j;
        a1 = i;
    else
        b1 = i;
        a1 = j;
    end

    phi = angleAP(V, b1, a1)-pi;
    gamma_b = angleAP(V, decrement(b1,n), b1);
    gamma_a = angleAP(V, decrement(a1,n), a1) - phi;
    if gamma_b < gamma_a 
        b2 = decrement(b1,n);
        a2 = a1;
    else
        b2 = decrement(a1,n);
        a2 = b1;
    end
    
    d1 = distPoint2Line(V(b1,:),V(increment(b1,n),:),V(a1,:));
    d2 = distPoint2Line(V(b2,:),V(increment(b2,n),:),V(a2,:));
    
    if d1 < d2
        b_vertex = b1;
        c_vertex = increment(b1,n);
        inclination = slopeAngle(V(b_vertex,:),V(c_vertex,:));       
             
        Pstart = V;
        Pend = circshift(V, -1);
%         figure; axis equal;
%         hold on;
%         line([Pstart(:,1)';Pend(:,1)'],[Pstart(:,2)';Pend(:,2)'],'Color','k');
%         scatter(Pstart(b1,1), Pstart(b1,2), 20, 'filled');
%         scatter(Pstart(c_vertex,1), Pstart(c_vertex,2), 20, 'filled');
%         title('Polygon in best path');
%         hold off;

        theta = pi/2 - inclination; % R rotation is with respect of the east; alpha rotation is with respect of the north
        
        PsR = rotatePolygon(Pstart, theta);
        PsR = PsR';
        PeR = rotatePolygon(Pend, theta);
        PeR = PeR';

%         figure; axis equal;
%         hold on;
%         line([PsR(:,1)';PeR(:,1)'],[PsR(:,2)';PeR(:,2)'],'Color','k');
%         scatter(PsR(b1,1), PsR(b1,2), 20, 'filled');
%         scatter(PsR(c_vertex,1), PsR(c_vertex,2), 20, 'filled');
%         title('Polygon rotated in best path');

        [PathRotated, transl_dist] = getPathMR([PsR PeR], dx, 1);
%         plot(PathRotated(:,1), PathRotated(:,2));
%         hold off;
%         pause;
        Path = rotatePolygon(PathRotated, -theta);
        Path = Path';
    else
        b_vertex = increment(b2,n);
        c_vertex = b2;
        inclination = slopeAngle(V(b_vertex,:),V(c_vertex,:));
%        inclination * 180 / pi
%        R(k) = inclination;

        Pstart = V;
        Pend = circshift(V, -1);
%         figure; axis equal;
%         hold on;
%         line([Pstart(:,1)';Pend(:,1)'],[Pstart(:,2)';Pend(:,2)'],'Color','k');
%         scatter(Pstart(b_vertex,1), Pstart(b_vertex,2), 20, 'filled');
%         scatter(Pstart(c_vertex,1), Pstart(c_vertex,2), 20, 'filled');
%         title('Polygon in best path inv');
%         hold off;

        theta = -pi/2 - inclination; % R rotation is with respect of the east; alpha rotation is with respect of the north
        PsR = rotatePolygon(Pstart, theta);
        PsR = PsR';
        PeR = rotatePolygon(Pend, theta);
        PeR = PeR';

%         figure; axis equal;
%         hold on;
%         line([PsR(:,1)';PeR(:,1)'],[PsR(:,2)';PeR(:,2)'],'Color','k');
%         scatter(PsR(b_vertex,1), PsR(b_vertex,2), 20, 'filled');
%         scatter(PsR(c_vertex,1), PsR(c_vertex,2), 20, 'filled');
%         title('Polygon rotated in best path inv');


        [PathRotated, transl_dist] = getPathMR([PsR PeR], dx, -1);
%         plot(PathRotated(:,1), PathRotated(:,2));
%         hold off;
%         pause;
        Path = rotatePolygon(PathRotated, -theta);
        Path = Path';
    end
end

function i_next = increment(i,n)
    i_next = mod(i,n)+1;
end

function i_prev = decrement(i,n)
    i_prev = i-1;
    if i_prev <1
        i_prev = n;
    end
end

function alpha = slopeAngle(p1, p2)
    alpha = atan2(p2(2)-p1(2),p2(1)-p1(1));
end
