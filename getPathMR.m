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

function [Path] = getPathMR(M, dx, dir)
% Returns a path for a given CONVEX polygon 
% This is for a multirotor UAV, namely it generates sharp corners
% Parameters: 
% M: plygon. M should be formed by:
% [px0 py0 px1 py1]
% [px1 py1 px2 py2]
% . 
% .
% [pxn pyn px0 py0]
% 
% dx: distance between fligh lines
% dir: direction of the initial flight line. 1: forth (abajo arriba), -1:
% back (arriba abajo). Default dir = 1% 

% Returns only the waypoints
% For computing distances use getPathMR2

%% Generate lines
gap_y = 1;

% polygon limits
l_limit = min(M(:,1));
r_limit = max(M(:,1));
down_limit = min(M(:,2));
up_limit = max(M(:,2));

% extend the lines beyond the polygon
y1 = down_limit - gap_y;
y2 = up_limit + gap_y;

x1 = l_limit + dx/2;
x2 = x1;

%dir = 1;% 1: forth (abajo arriba), -1: back (arriba abajo)

%% Show the intersection points.
%     figure('Position',[10 100 500 500],'Renderer','zbuffer'); 
%     axes_properties.box             = 'on';
%     axes_properties.XLim            = [-2 2];
%     axes_properties.YLim            = [-2 2];
%     axes_properties.DataAspectRatio = [1 1 1];
%     axes_properties.NextPlot        = 'add';
%     axes(axes_properties,'parent',gcf);
%     hold on
  
%%

i = 1; %numero de waypoints agregados
lines = 0;
%nLinesF = 0;
%nLinesB = 0;
enterWP = [0 0];
exitWP = [0 0];
lastWP = [0 0];

while(x1 < (r_limit + dx/2)) %% TEST WARNING!!! previous: while(x1 <= r_limit)
    % linea de intersección
    LineXY = [x1 y1 x2 y2];
    lines = lines +1;
    
    %intersecciones
    out = lineSegmentIntersect(M,LineXY);
    intersections = [out.intMatrixX(:) out.intMatrixY(:)];
    intersections( ~any(intersections,2), : ) = [];  
    n = size(intersections,1);  
    
    % Si solo hay una intersección agregamos el punto directamente
    if (n == 1)
        p1 = intersections(1,:);
        Path(i,:) = p1;
        lastWP = p1;
        i = i + 1;
    end
    
    % si hay dos intersecciones entonces tomamos las intersecciones como wp
    if (n > 1)
        
        if(n>2)
           [B, kkk] = sort(intersections(:,2)); % TODO: Revisar que esto este bien
           ordered = [intersections(kkk) B];
           p1 = ordered(1,:);
           p2 = ordered(n,:);
        else
            p1 = intersections(1,:);
            p2 = intersections(2,:);
        end
            
        % las ordenamos de acuerdo a la dirección
        if (dir == 1) % direcciń hacia arriba
            
            % ordenar los puntos para que corresponda con la dirección
            % hacia arriba
            if (p2(1,2)<p1(1,2))
                enterWP = p2; % punto de entrada a la linea
                exitWP = p1; % punto de salida de la linea
            else
                enterWP = p1;
                exitWP = p2;
            end
            
            % si hay lineas previas entonces ajustamos los nuevos waypoints con los waypoints the la linea previa
            if(lines > 1)
                dif = enterWP(1,2) - lastWP(1,2);
                if(dif < 0) 
                    % Si la diferencia es negativa entonces el último punto
                    % está arriba del punto de entrada.
                    % Entones reemplazar el último punto.
                    intermediateWP = [lastWP(1,1) enterWP(1,2)];
                    Path(i-1,:) = intermediateWP;
                else
                    % De lo contrario, mover el punto de entradada
                    intermediateWP = [enterWP(1,1) lastWP(1,2)];
                    enterWP = intermediateWP;
                end
            end
            
            % Agregar puntos de la nueva linea
            Path(i,:) = enterWP;
            i = i+1;
            Path(i,:) = exitWP;
            i = i+1;
            
        else % dirección hacia abajo
           %ordenar los waypoints
           if (p2(1,2)<p1(1,2)) 
               enterWP = p1;
               exitWP = p2;
           else
               enterWP = p2;
               exitWP = p1;
           end
           
           if(lines > 1)
                % Alinear el ultimo punto y el punto de entrada
                dif = enterWP(1,2) - lastWP(1,2);
                if(dif > 0) 
                    % Modificar el ultimo punto
                    intermediateWP = [lastWP(1,1) enterWP(1,2)];
                    Path(i-1,:) = intermediateWP;
                else
                    % Modificar el punto de entrada
                    intermediateWP = [enterWP(1,1) lastWP(1,2)];
                    enterWP = intermediateWP;      
                end
            end
            Path(i,:) = enterWP;
            i = i+1;
            Path(i,:) = exitWP;
            i = i+1;

        end
        
        dir = dir * -1;
        lastWP = exitWP;
    end
    
    % Si no hay intersecciones ya salio del polígono pero una parte del
    % polígono no ha sido vista por la mitad del FOV
    if (n==0)
        %regresamos la linea una distancia igual a la mitad del FOV
        % linea de intersección
        LineFOV = [x1-dx/2 y1 x2-dx/2 y2];
        
        % Verificar la intersección del FOV
        %intersecciones
        out = lineSegmentIntersect(M, LineFOV);
        intersections = [out.intMatrixX(:) out.intMatrixY(:)];
        intersections( ~any(intersections,2), : ) = [];  
        n = size(intersections,1);
        
        %si hubo mas de una intersección
        if(n > 1)
            
            if(n>2)
               [B, kkk] = sort(intersections(:,2)); % TODO: Revisar que esto este bien
               ordered = [intersections(kkk) B];
               p1 = ordered(1,:);
               p2 = ordered(n,:);
            else
                p1 = intersections(1,:);
                p2 = intersections(2,:);
            end

            % ajustar las intersecciones para que la linea que se agrege
            % quede fuera del poligono
            p1(1,1) = p1(1,1) + dx/2;
            p2(1,1) = p2(1,1) + dx/2;

            % las ordenamos de acuerdo a la dirección
            if (dir == 1) % direcciń hacia arriba

                % ordenar los puntos para que corresponda con la dirección
                % hacia arriba
                if (p2(1,2)<p1(1,2))
                    enterWP = p2; % punto de entrada a la linea
                    exitWP = p1; % punto de salida de la linea
                else
                    enterWP = p1;
                    exitWP = p2;
                end

                % si hay lineas previas entonces ajustamos los nuevos waypoints con los waypoints the la linea previa
                if(lines > 1)
                    dif = enterWP(1,2) - lastWP(1,2);
                    if(dif < 0) 
                        % Si la diferencia es negativa entonces el último punto
                        % está arriba del punto de entrada.
                        % Entones reemplazar el último punto.
                        intermediateWP = [lastWP(1,1) enterWP(1,2)];
                        Path(i-1,:) = intermediateWP;
                    else
                        % De lo contrario, mover el punto de entradada
                        intermediateWP = [enterWP(1,1) lastWP(1,2)];
                        enterWP = intermediateWP;
                    end
                end

                % Agregar puntos de la nueva linea
                Path(i,:) = enterWP;
                i = i+1;
                Path(i,:) = exitWP;
                i = i+1;

            else % dirección hacia abajo

               % ordenar los waypoints
               if (p2(1,2)<p1(1,2)) 
                   enterWP = p1;
                   exitWP = p2;
               else
                   enterWP = p2;
                   exitWP = p1;
               end

               if(lines > 1)
                    % Alinear el ultimo punto y el punto de entrada
                    dif = enterWP(1,2) - lastWP(1,2);
                    if(dif > 0) 
                        % Modificar el ultimo punto
                        intermediateWP = [lastWP(1,1) enterWP(1,2)];
                        Path(i-1,:) = intermediateWP;
                    else
                        % Modificar el punto de entrada
                        intermediateWP = [enterWP(1,1) lastWP(1,2)];
                        enterWP = intermediateWP;      
                    end
                end
                Path(i,:) = enterWP;
                i = i+1;
                Path(i,:) = exitWP;
                i = i+1;

            end

            dir = dir * -1;
            lastWP = exitWP;
        end
    end
    
    %nLines = [nLinesB nLinesF];
    %pause
    
    x1 = x1 + dx;
    x2 = x1;
end

% title('Intersection Points');
% hold off;

end