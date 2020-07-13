clear;
addpath UAV_Mapping_v3
addpath coloring
addpath test
addpath GNSSLIB
addpath thermo_original
addpath wideM
addpath RUSTmask
addpath PIPEmask
addpath INmaporig
addpath LIGHTmask
addpath dictthermo
addpath RGB

%“Ç‚İ‚İ
map_original = importdata('pointclouds_16line.pts');
map=map_original(:,1:3);
center=[-51.875 -29.461 3.7904];

%2019”N“xŠî€‹Ç‚©‚ç‚Ì‹——£
%2019
%basemove=[0 0 0];

%2018
basemove=[-0.5055 -2.528 0.1817];


%map_dot=map-repmat(center,length(map),1)-repmat(basemove,length(map),1

map_dot=map-repmat(center,length(map),1);
map_trans_dot=rpy2rotM(0,0,-152.68)*map_dot';
map_trans=map_trans_dot';
mapT=horzcat(map_trans,map_original(:,4));


dlmwrite('pointclouds_2018.pts', mapT, 'delimiter', ' ');

dlmwrite('pointclouds_2018.csv', mapT, 'delimiter', ' ');

