% clc; clear all; close all;

%Active directories
image_dir = '/media/gokulbnr/GBN - My Passport/RRC/Datasets/KITTI/data_tracking_image_2/training/image_02';
label_dir = '/media/gokulbnr/GBN - My Passport/RRC/Datasets/KITTI/data_tracking_label_2/training/label_02';
[status,cmdout] = system('pwd');
cmdout = strsplit(cmdout,'\n');

root_dir  = cmdout{1};
result_dir = sprintf('%s/ResultsNewMulti/%d_%d_%d-%d/',root_dir,seq,carID,fir,las);
system(sprintf('mkdir -p %s',result_dir));

%Camera Intrinsic Parameter
K = [721.53,0,609.55;0,721.53,172.85;0,0,1];

%Dataset sequence and frame initialization
dat = importdata(sprintf('infofile.txt'));
seq = dat(:,2);
frm = dat(:,3);
id  = dat(:,4);

avgCarHeight = 1.5208;
avgCarWidth = 1.6362;
avgCarLength = 3.8600;

carInfo = [];

%Get parameters for the noted car id's
for idx = fir:las
    tracklets = readLabels(label_dir,seq(idx));
    t_frm = tracklets{frm(idx)+1};
    for i = 1:size(t_frm,2)
        if t_frm(i).id == id(idx)
            temp=[double(t_frm(i).frame),double(t_frm(i).id),double(t_frm(i).x1),double(t_frm(i).y1),double(t_frm(i).x2),double(t_frm(i).y2),double(t_frm(i).ry)];
            carInfo=[carInfo;temp];
        end
    end
end

% Load Keypoints
kpsa = importdata(sprintf('result_KP.txt'));
NumKeypoints = size(kpsa,2)/3;

% Load MeanShape and Scale it to mean dimensions
meanShape = importdata('meanShape.txt');
meanShape = meanShape';
average_dimensions = [avgCarLength;avgCarHeight;avgCarWidth];
scale_from = [max(meanShape(1,:))-min(meanShape(1,:)) ; %length
              max(meanShape(2,:))-min(meanShape(2,:)) ; %height
              max(meanShape(3,:))-min(meanShape(3,:))]; %width
scale_factor = average_dimensions./scale_from;
meanShapeScaled = diag(scale_factor) * meanShape;

% Load deformation vectors and scale them to mean dimensions
vectors = importdata('vectors.txt');
NumVectors = size(vectors,1);
for j=1:NumVectors
    mat = reshape(vectors(j,:),3,NumKeypoints);
    sca = diag(scale_factor) * mat;
    vectors(j,:) = reshape(mat,1,3*NumKeypoints);
end
    
%%%%%%%%%%%%%%%%%%%%%%Temporary%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cd Ceres
% system('make clean');
% system('make');
% cd ..
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Kp Lookup for binary weights based on self occlusions
kp_lookup = importdata('kpLookup_azimuth.mat');

outfile = sprintf('%sout%d.txt',result_dir,subseq);
fid = fopen(outfile,'w');

SHAPE = {};KPS = {};CENTRE={};VECTORS={};POSEafterPnP={};LaftersingleView={};SHAPEbeforesingleView={};SHAPEaftersingleView={};VECTORSaftersingleView={};
v = 1;

ini_vectors = vectors;

%Iterate over dataset
 for i=fir:las
     if dat(i,4) ~= carID
         continue;
     end
    SHAPE{v} = meanShapeScaled;
%     VECTORS{v} = vectors;
    
    i
    %Get keypoints
    kps = kpsa(i,:);
    kps = reshape(kps,3,NumKeypoints);

    for j=1:36
        if kps(3,j) <0
            kps(3,j) = 0.01;
        end
    end
    
    %Scale from 64X64 to bbox size and translate to the bbox position
    for j=1:size(kps,2)
        kps(1,j) = carInfo(i-fir+1,3)+((carInfo(i-fir+1,5)-carInfo(i-fir+1,3)+1)*kps(1,j))/64;
        kps(2,j) = carInfo(i-fir+1,4)+((carInfo(i-fir+1,6)-carInfo(i-fir+1,4)+1)*kps(2,j))/64;
    end
    
    %Plot keypoints
    filename = sprintf('%s/%04d/%06d.png',image_dir,seq(i),frm(i));
    img = imread(filename);
%     f=figure('visible','off');     %hold on; % Temporary
%     subplot(2,2,1);
%     imshow(img); hold on;
%     scatter(kps(1,:),kps(2,:),'filled');
%     title('Keypoints');
    
    %Rotate meanShape based on the given 'ry' (azimuth) parameter
    theta_y = mod(270 + rad2deg(carInfo(i-fir+1,7)) + 10*rand,360);
%     theta_y = 270 + rad2deg(carInfo(i,7)) + 10*rand;
    meanShapeScaledRotated = roty(theta_y) * rotz(180) * roty(90) * meanShapeScaled;
    
    %Rotate vectors based on the given 'ry' (azimuth) parameter
    for j=1:NumVectors
        mat = reshape(vectors(j,:),3,NumKeypoints);
        mat = roty(theta_y) * rotz(180) * roty(90) * mat;
        vectors(j,:) = reshape(mat,1,3*NumKeypoints);
    end
    
%     VECTORS{v} = vectors;
    
    kps = kps';
    wV = kp_lookup(ceil(theta_y),:)/(sum(kp_lookup(ceil(theta_y),:)));
    wHG = kps(:,3);
    kps(:,3) = 0.7*(wV') + 0.3*wHG ;
    kps = kps';
    KPS{v} = kps';
    
    %Get car centre
    bbox_bottom = [(carInfo(i-fir+1,3)+carInfo(i-fir+1,5))/2;carInfo(i-fir+1,6);1];
    ground_normal = [0;-1;0];
    carCentre = -(avgCarHeight * inv(K) * bbox_bottom) / (ground_normal' * inv(K) * bbox_bottom);
    carCentre(3) = carCentre(3) + avgCarLength/2;
    carCentre(2) = carCentre(2) - avgCarHeight/2;
    CENTRE{v} = carCentre;
    
    %Translate meanShape from camera centre to car centre
    meanShapeScaledRotatedTranslated = meanShapeScaledRotated + repmat(carCentre,1,NumKeypoints);
    SHAPEbeforesingleView{v} = meanShapeScaledRotatedTranslated';
    
    %Plot meanShape
    m = K * meanShapeScaledRotatedTranslated;
    m(1,:) = m(1,:)./m(3,:);
    m(2,:) = m(2,:)./m(3,:);
%     subplot(2,2,2);
%     figure,
%     visualizeWireframe2D(img,m(1:2,:));
%     title('Initial Wireframe');
    
    %Make input file for pose adjuster
    write_inp_pose(carCentre,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanShapeScaledRotatedTranslated,vectors, 3*pi/2 + carInfo(i-fir+1,7),kp_lookup);
    
    %Run singleViewPoseAdjuster
    cd Ceres
    system('./singleViewPoseAdjuster');
    cd ..
    
    pos = importdata('Ceres/ceres_output_singleViewPoseAdjuster.txt');
    R = [pos(1),pos(2),pos(3);pos(4),pos(5),pos(6);pos(7),pos(8),pos(9)]';
    t = [pos(10);pos(11);pos(12)];
    POSEafterPnP{v} = pos;
    
    tempt=R*carCentre+t;
    tempr = R*roty(theta_y) * rotz(180) * roty(90);
%     fprintf(fid,'%d %d %d %f %f %f %f %f %f %f %f %f %f %f %f\n',seq(i),frm(i),id(i),tempr(1),tempr(2),tempr(3),tempr(4),tempr(5),tempr(6),tempr(7),tempr(8),tempr(9),tempt(1),tempt(2),tempt(3));
    
    m = R * meanShapeScaledRotatedTranslated;
    m = m + repmat(t,1,NumKeypoints);
    m = K * m;
    m(1,:) = m(1,:)./m(3,:);
    m(2,:) = m(2,:)./m(3,:);
%     subplot(2,2,3);
%     visualizeWireframe2D(img,m(1:2,:));
%     visualizeWireframe2DandSaveImage(img,m(1:2,:),v+100);
%     title('After Pose Adjustment');
    
    %Make input file for shape adjuster
    write_inp_shape(carCentre,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanShapeScaledRotatedTranslated,vectors, 3*pi/2 + carInfo(i-fir+1,7) ,kp_lookup);
    
    %Run singleViewShapeAdjuster
    cd Ceres
    system('./singleViewShapeAdjuster');
    cd ..
    
    %Plot final result
    wireFrame = importdata('Ceres/ceres_output_singleViewShapeAdjuster.txt');
    wireFrame = wireFrame';
    
%     tempx = (wireFrame(1,1) + wireFrame(1,2) + wireFrame(1,19) + wireFrame(1,20))/4;
%     tempy = (wireFrame(2,1) + wireFrame(2,2) + wireFrame(2,19) + wireFrame(2,20))/4;
%     tempz = (wireFrame(3,1) + wireFrame(3,2) + wireFrame(3,19) + wireFrame(3,20))/4;
% 
%     fprintf(fid,'%d %d %d %f %f %f %f %f %f %f %f %f %f %f %f\n',seq(i),frm(i),id(i),tempr(1),tempr(2),tempr(3),tempr(4),tempr(5),tempr(6),tempr(7),tempr(8),tempr(9),tempx,tempy,tempz);

    wireFrame = K * wireFrame;
    wireFrame(1,:) = wireFrame(1,:)./wireFrame(3,:);
    wireFrame(2,:) = wireFrame(2,:)./wireFrame(3,:);
%     subplot(2,2,4)
%     visualizeWireframe2D(img,wireFrame(1:2,:));
%     visualizeWireframe2DandSaveImage(img,wireFrame(1:2,:),v,result_dir);
%     title('After Shape Adjustment');
%     close all;
%     suptitle(sprintf('Image %d',i));
    
    l = importdata('Ceres/lvals.txt');
    LaftersingleView{v} = l;
    
    ve = importdata('Ceres/vvals.txt');
    VECTORSaftersingleView{v} = ve;
    
    v = v+1;
 end
 
numViews = v-1;

for i=1:numViews
    VECTORS{i}=vectors;
end

write_inpFile_multiviewadjuster;
numViews

cd Ceres/
system('./multiViewShapeandPoseAdjuster');
cd ..

untitled;

fclose(fid);
