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

%読み込み
map = importdata('pointclouds.pts');
flag= importdata('flag_pointcloud2.pts');
posatt=importdata("posatt.csv");

%点群飛行パラメータ
pos=posatt(:,2:4);
UAV_qua =zeros(1,4);
UAV_pos = zeros(1,3);
KK = zeros(3, 3);
kc = zeros(5, 1);
Xn = zeros(2, length(map));
Xp = zeros(3, length(map));
Xp_round=zeros(3, length(map));
%colored_map = horzcat(map(:, 1:3), zeros(length(map), 4));
colored_map_dot = horzcat(map(:, 1:4), zeros(length(map), 4));

%画像パラメータ
image_vertical = 512;
image_width = 640;
image_focus = 19;
image_num = 129;
tods = hms2sec([02 31 5.6]); 
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
RGBend=966;

N=(-RGBstart+RGBend)/2+1;

%温度パラメータ
% NEWthermo=zeros(image_vertical,image_width);
% Esabi=0.95;
% Eb=0.65;
% 
% Cloud=4;
% Tatm=8;
% rh=50;
% 
% K=1+0.0224*Cloud-0.0035*Cloud^2+0.00028*Cloud^3;
% ews=rh*0.01*exp((17.502*Tatm)/(240.9+Tatm));
% Tdew=(240.9*log(ews))/(17.5-log(ews));
% Tsky=(273.15+Tatm)*((0.787+0.764*log((Tdew+273.15)/273.15))*K)^0.25-273.15;



VerrorM=zeros(N,1);
WerrorM=zeros(N,1);




%画像ごとの処理
%for n = 1:N
for n = 63:63
    
%温度補正
%     sabimask=imread(strcat("sabimaskDJI_0",num2str(2*(n-1)+RGBstart),".jpg"));
%     tha=importdata(strcat("thermo",num2str(2*(n-1)+RGBstart),".csv"));
%     thermo=tha.data(1:512,1:640);
%     
%     for i=1:512
%     for j=1:640
%         if (sabimask(i,j)==0)
%             NEWthermo(i,j)=((Eb*(thermo(i,j)+273.15)^4+(1-Eb)*(Tsky+273.15)^4-(1-Esabi)*(Tsky+273.15)^4)/Esabi)^0.25-273.15;
%         else
%             NEWthermo(i,j)=thermo(i,j);
%             
%         end
%     end
%     end
%     dlmwrite(strcat("dictthermo/dictthermo",num2str(2*(n-1)+RGBstart),".csv"), NEWthermo, 'delimiter', ',');

  %補正温度持ってるなら  
    NEWthermo=importdata(strcat("dictthermo",num2str(2*(n-1)+RGBstart-1),".csv"));
   
    



%点群付加
    image_RGB = importdata(strcat('DJI_0', num2str(2*n+RGBstart-2),'.jpg'));
    image_thermo = NEWthermo;
    
    t=tods+18+image_dsec*(n-1);
    UAV_quan = find(posatt(:,1)==t);
    UAV_qua=posatt(UAV_quan,5:8);
    UAV_rpyd=quo2rpy(UAV_qua);
    UAV_rpyn=zeros(1,3);
    UAV_rpyn(1,3)=UAV_rpyd(1,3);
    
    if UAV_rpyn(1,3)>0
    UAV_rpyn(1,3)=UAV_rpyn(1,3)-180;
    else
    UAV_rpyn(1,3)=UAV_rpyn(1,3)+180;
    end

    
    
    UAV_pos = posatt(UAV_quan,2:4);
    
    %UAV⇒点群（ENU座標）
    XX=map(:, 1:3)-repmat(UAV_pos(1,:),length(map),1);
    %UAV⇒点群（UAV座標）
    XX_dot=qua2rotM(UAV_qua)*rpy2rotM(0,0,180)*XX.';
    %XX_dot=rpy2rotM(UAV_rpyn(1,1),UAV_rpyn(1,2),UAV_rpyn(1,3))*rpy2rotM(0,0,180)*XX.';
    %カメラ⇒点群（UAV座標）
    XXc_dot=XX_dot-repmat(cam_p',1,length(map));
    %カメラ⇒点群（カメラ座標）
    XXc=rpy2rotM(0,180,0)*XXc_dot;
    
   
    Xn(1,:) = XXc(1,:)./(XXc(3,:));
    Xn(2,:) = XXc(2,:)./(XXc(3,:));
    
    Xp_dot = KK * vertcat(Xn,ones(1,length(Xn)));
    Xp(1,:)=Xp_dot(1,:);
    Xp(2,:)=Xp_dot(2,:);
    Xp(3,:)=XXc(3,:);
    
    Xp_round(1,:)=round(Xp(1,:));
    Xp_round(2,:)=round(Xp(2,:));
    Xp_round(3,:)=flag(:,4);
    
    
    
    wideM=importdata(strcat("DJI_0",num2str(2*(n-1)+RGBstart),"lineM.csv"));
    
    %tha=importdata(strcat("thermo",num2str(2*(n-1)+RGBstart),".csv"));
    %横線あったら
    if wideM(1,1)==1

        inten=zeros(image_vertical+550,image_width+200);
        
        coloredpoints_num = find(Xp_round(1,:) > -100 & Xp_round(2,:) > -275 & Xp_round(1,:) < image_width+100 & Xp(2,:) < image_vertical+275);
        Xp_round_using=Xp_round(:,coloredpoints_num);
        
        %2次元マップ
        for i=1:length(Xp_round_using)
            x= Xp_round_using(1,i);
            y= Xp_round_using(2,i);
            inten(y+275,x+100)=Xp_round_using(3,i);
        end
        
        
        
        
        %全探索
        searchM=zeros(110000,1);
        
        for w=1:200
            for v=1:550
                inM=inten(v:image_vertical+v-1,w:image_width+w-1);
                RGB=importdata(strcat("lightmaskDJI_0",num2str(2*(n-1)+RGBstart),".jpg"))==0;
                sum=inM+RGB;
                searchM((v-1)*200+w,1)=length(find(sum==2));
            end
        end
        
        [M,I]=max(searchM);
        R=rem(I,200);
        Q=fix(rdivide(I,200));
        WerrorM(n,1)=-R+100;
        VerrorM(n,1)=-Q+275;
        
        
       
        
        
    %横線なかったら    
    else
        inten=zeros(image_vertical,image_width+200);
        
        coloredpoints_num = find(Xp_round(1,:) > -100 & Xp_round(2,:) > 0 & Xp_round(1,:) < image_width+100 & Xp(2,:) < image_vertical);
        Xp_round_using=Xp_round(:,coloredpoints_num);
        
        %2次元マップ
        for i=1:length(Xp_round_using)
            x= Xp_round_using(1,i);
            y= Xp_round_using(2,i);
            inten(y,x+100)=Xp_round_using(3,i);
        end
        
       
        
        %全探索
        searchM=zeros(200,1);
        
%         for w=1:200
%             inM=inten(:,w:image_width+w-1);
%             RGB=importdata(strcat("pipemaskDJI_0",num2str(2*(n-1)+RGBstart),".jpg"))==0;
%             sum=inM+RGB;
%             searchM(w,1)=length(find(sum==2));
%         end
        
        [M,I]=max(searchM);
%         WerrorM(n,1)=-I+100;
        WerrorM(n,1)=-23;
        if n==63
            VerrorM(n,1)=-150;
        else
            VerrorM(n,1)=VerrorM(n-1,1);
        end
        
    end
    
    Verror=VerrorM(n,1);
    Werror=WerrorM(n,1);
    paint_num = find(Xp_round(1,:) > 0-Werror & Xp_round(2,:) > -Verror & Xp_round(1,:) < image_width-Werror & Xp_round(2,:) < image_vertical-Verror);
    
    %シンプル色付け
    for i = 1:length(paint_num)
        colored_map_dot(paint_num(1,i),5) = image_RGB(Xp_round(2,paint_num(1,i))+Verror,Xp_round(1,paint_num(1,i))+Werror,1);
        colored_map_dot(paint_num(1,i),6) = image_RGB(Xp_round(2,paint_num(1,i))+Verror,Xp_round(1,paint_num(1,i))+Werror,2);
        colored_map_dot(paint_num(1,i),7) = image_RGB(Xp_round(2,paint_num(1,i))+Verror,Xp_round(1,paint_num(1,i))+Werror,3);
        colored_map_dot(paint_num(1,i),8) = image_thermo(Xp_round(2,paint_num(1,i))+Verror,Xp_round(1,paint_num(1,i))+Werror);    
    end
end

%空白飛ばし
nonzeroP=find(colored_map_dot(:,8));
colored_map=zeros(length(nonzeroP),8);

for c=1:length(nonzeroP)
    colored_map(c,:)=colored_map_dot(nonzeroP(c,1),:);
end

%座標変換
mapA=colored_map(:,1:3);
center=[-51.875 -29.461 3.7904];

map_dot=mapA-repmat(center,length(mapA),1);
map_trans_dot=rpy2rotM(0,0,-152.68)*map_dot';
map_trans=map_trans_dot';
mapT=horzcat(map_trans,colored_map(:,4:8));

%dlmwrite('error1204.csv', verrorM, 'delimiter', ' ');
dlmwrite('Coloredmap_847.pts', mapT, 'delimiter', ' ');