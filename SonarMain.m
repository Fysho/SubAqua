sonarProces = SonarProcess(0.65);
sonarData = csvread("2020_10_15_8h21m36s.csv");
sonarData = csvread("2020_10_15_5h24m34s.csv");

flatimage = sonarProces.ConvolveImage(sonarData,3);
imshow(sonarData/255);

% heatmap = sonarProces.CreateHeatMap(flatimage*255);
% heatmap = sonarProces.CreateHeatMap(sonarData);

% imshow(heatmap);
