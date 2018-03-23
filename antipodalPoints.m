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

% Computes all antipodal pairs
% Receives a poygon in P, the polygon must be clock wise ordered 
% [px0 py0]
% .
% [pxn pyn]

function A = antipodalPoints(P)
    % Find an initial antipodal pair by locating the vertex opposite p1
    [n, ~] = size(P);
    i = 1;
    j = 2;
    A =[];

    while angleAP(P, n, j) < pi
        j = increment(j,n);
    end
    A = [A; i j];

    while j<=n
        %if (angleAP(P, i, j) <= angleAP(P, j, i))
        diff = pi-angleAP(P,i,j);
        if (diff>0)
            j = j+1;
        else
            if (diff<0)
                i = i+1;
            else
                if(i+1<j)
                    A = [A;increment(i,n) j];
                end
                if(j+1<=n)
                    A = [A;i increment(j,n)];
                end                
                i = i+1;
                j = j+1;
            end
        end
        if(j<=n && i<j)
            A = [A; i j];
        end
    end
end

function i_next = increment(i,n)
    i_next = mod(i,n)+1;
end

