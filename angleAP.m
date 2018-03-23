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

function angle = angleAP(P, i, j)
    [n, ~]= size(P);
    
    i_next = mod(i,n)+1; 
    y = P(i_next,2) - P(i,2);
    x = P(i_next,1) - P(i,1);
    alpha_i = atan2(y,x);
    
    j_next = mod(j,n)+1;
    y = P(j_next,2) - P(j,2);
    x = P(j_next,1) - P(j,1);
    alpha_j = atan2(y,x);

    angle = clockWiseDist(alpha_i, alpha_j);

end

% clock wise distance from a to b
function angle = clockWiseDist(a , b)
   if a<0
       if b<0
           if(a<b)
               angle = 2*pi + (a-b);
           else
               angle = a - b;
           end
       else
           a = 2*pi + a;
           angle = a-b;
       end
   else
       if b<0
           angle = a - b;
       else
           if(a<b)
               angle = 2*pi-(a-b);
           else
               angle = a-b;
           end
       end
   end
end