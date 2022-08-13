% Loads the polygon from a CSV file

function [Polygon, shifted_Polygon] = loadPolygonCSV(file)
    Polygon = readmatrix(file);
    shifted_Polygon = circshift(Polygon, -1);
end