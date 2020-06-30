clear;
addpath UAV_Mapping_v3
addpath coloring
addpath test
addpath GNSSLIB
addpath thermo_original
addpath wideM
addpath INmaporig
addpath RUSTmask
addpath PIPEmask
addpath LIGHTmask
addpath dictthermo
addpath RGB



%画像パラメータ
image_vertical = 512;
image_width = 640;
image_focus = 19;
image_num = 129;
tods = hms2sec([06 56 21.8]); 
image_dsec = 2;
%UAV⇒カメラのベクトル（UAV座標）
cam_p = [0.0 0.15 0.0];
%カメラ行列
KK(1,3) = 295; 
KK(2,3) = 247; 
KK(1,1) = 1180;
KK(2,2) = 1130;
KK(3,3) = 1;

RGBstart=750;
RGBend=996;

N=(-RGBstart+RGBend)/2+1;

%温度パラメータ
NEWthermo=zeros(image_vertical,image_width);
Esabi=0.95;
Eb=0.65;

Cloud=5;
Tatm=8;
rh=50;

K=1+0.0224*Cloud-0.0035*Cloud^2+0.00028*Cloud^3;
ews=rh*0.01*exp((17.502*Tatm)/(240.9+Tatm));
Tdew=(240.9*log(ews))/(17.5-log(ews));
Tsky=(273.15+Tatm)*((0.787+0.764*log((Tdew+273.15)/273.15))*K)^0.25-273.15;



VerrorM=zeros(N,1);
WerrorM=zeros(N,1);




%画像ごとの処理
for n = 1:N
%for n = 114:118

%温度補正
    sabimask=imread(strcat("sabimaskDJI_0",num2str(2*(n-1)+RGBstart),".jpg"));
    tha=importdata(strcat("thermo",num2str(2*(n-1)+RGBstart-1),".csv"));
    thermo=tha.data(1:512,1:640);
    
    for i=1:512
    for j=1:640
        if (sabimask(i,j)==0)
            NEWthermo(i,j)=((Eb*(thermo(i,j)+273.15)^4+(1-Eb)*(Tsky+273.15)^4-(1-Esabi)*(Tsky+273.15)^4)/Esabi)^0.25-273.15;
        else
            NEWthermo(i,j)=thermo(i,j);
            
        end
    end
    end
    dlmwrite(strcat("dictthermo/dictthermo",num2str(2*(n-1)+RGBstart-1),".csv"), NEWthermo, 'delimiter', ',');
    
end
