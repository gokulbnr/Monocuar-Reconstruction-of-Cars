clc; clear all; close all;

%Active directories
root_dir = '/home/gokulbnr/RRC/IROS18';
label_dir = '/home/gokulbnr/RRC/KITTI/training/label_02';
image_dir = '/home/gokulbnr/RRC/KITTI/data_tracking_image_2/training/image_02';

%Camera Intrinsic Parameter
K = [721.53,0,609.55;0,721.53,172.85;0,0,1];

%Dataset sequence and frame initialization
seq = [  1,  1,  1,  1,  1,  1 ];
frm = [ 65, 66, 67, 68, 69, 70 ];
id  = [ 13, 13, 13, 13, 13, 13 ];

avgCarHeight = 1.5208;
avgCarWidth = 1.6362;
avgCarLength = 3.8600;

carInfo = [];
error = [];
e = [];

%Get parameters for the noted car id's
for idx = 1:6
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
kpsa = importdata('keypoints.mat');
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
cd Ceres
system('make clean');
system('make');
cd ..
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Iterate over dataset
 for i=1:6
    
    %Get keypoints
    kps = kpsa(i,:);
    kps = reshape(kps,3,NumKeypoints);
%     if i==3
%         kps(3,6) = 0.6;
%     elseif i==1
%         kps(3,5) = 0.6;
%     end
    
    %Scale from 64X64 to bbox size and translate to the bbox position
    for j=1:size(kps,2)
        kps(1,j) = carInfo(i,3)+((carInfo(i,5)-carInfo(i,3)+1)*kps(1,j))/64;
        kps(2,j) = carInfo(i,4)+((carInfo(i,6)-carInfo(i,4)+1)*kps(2,j))/64;
    end
    
    %Plot keypoints
    filename = sprintf('%s/%04d/%06d.png',image_dir,seq(i),frm(i));
    img = imread(filename);
    figure,
    subplot(2,2,1);
    imshow(img); hold on;
    scatter(kps(1,:),kps(2,:),'filled');
    title('Keypoints');
    
    %Rotate meanShape based on the given 'ry' (azimuth) parameter
%     theta_y = mod(270 + rad2deg(carInfo(i,7)) + 10*rand,360);
    theta_y = 270 + rad2deg(carInfo(i,7)); %+ 10*rand;
    meanShapeScaledRotated = roty(theta_y) * rotz(180) * roty(90) * meanShapeScaled;
    
    %Rotate vectors based on the given 'ry' (azimuth) parameter
    for j=1:NumVectors
        mat = reshape(vectors(j,:),3,NumKeypoints);
        mat = roty(theta_y) * rotz(180) * roty(90) * mat;
        vectors(j,:) = reshape(mat,1,3*NumKeypoints);
    end
    
    %Get car centre
    bbox_bottom = [(carInfo(i,3)+carInfo(i,5))/2;carInfo(i,6);1];
    ground_normal = [0;-1;0];
    carCentre = -(avgCarHeight * inv(K) * bbox_bottom) / (ground_normal' * inv(K) * bbox_bottom);
    carCentre(3) = carCentre(3) + avgCarLength/2;
    carCentre(2) = carCentre(2) - avgCarHeight/2;
    
    %Translate meanShape from camera centre to car centre
    meanShapeScaledRotatedTranslated = meanShapeScaledRotated + repmat(carCentre,1,NumKeypoints);
    
    %Plot meanShape
    m = K * meanShapeScaledRotatedTranslated;
    m(1,:) = m(1,:)./m(3,:);
    m(2,:) = m(2,:)./m(3,:);
    subplot(2,2,2);
%     figure,
    visualizeWireframe2D(img,m(1:2,:));
    title('Initial Wireframe');
    
    temp = 0;
    for j = 1:NumKeypoints
        if kps(3,j) > 0.2
        temp = temp + (m(1,j) - kps(1,j)) * (m(1,j) - kps(1,j)) + (m(2,j) - kps(2,j)) * (m(2,j) - kps(2,j));
        end
    end
    e = temp;
    
    %Kp Lookup for binary weights based on self occlusions
    kp_lookup = importdata('kpLookup_azimuth.mat');
    
    %Make input file for pose adjuster
    write_inp_pose(carCentre,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanShapeScaledRotatedTranslated,vectors, 3*pi/2 + carInfo(i,7),kp_lookup);
    
    %Run singleViewPoseAdjuster
    cd Ceres
    system('./singleViewPoseAdjuster');
    cd ..
    
    pos = importdata('Ceres/ceres_output_singleViewPoseAdjuster.txt');
    R = [pos(1),pos(2),pos(3);pos(4),pos(5),pos(6);pos(7),pos(8),pos(9)]';
    t = [pos(10);pos(11);pos(12)];
    
    m = R * meanShapeScaledRotatedTranslated;
    m = m + repmat(t,1,NumKeypoints);
    m = K * m;
    m(1,:) = m(1,:)./m(3,:);
    m(2,:) = m(2,:)./m(3,:);
    subplot(2,2,3);
    visualizeWireframe2D(img,m(1:2,:));
    title('After Pose Adjustment');
    
    temp = 0;
    for j = 1:NumKeypoints
        if kps(3,j) > 0.2
        temp = temp + (m(1,j) - kps(1,j)) * (m(1,j) - kps(1,j)) + (m(2,j) - kps(2,j)) * (m(2,j) - kps(2,j));
        end
    end
    e = [e;temp];
    
    %Make input file for shape adjuster
    write_inp_shape(carCentre,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanShapeScaledRotatedTranslated,vectors, 3*pi/2 + carInfo(i,7) ,kp_lookup);
    
    %Run singleViewShapeAdjuster
    cd Ceres
    system('./singleViewShapeAdjuster');
    cd ..
    
    %Plot final result
    wireFrame = importdata('Ceres/ceres_output_singleViewShapeAdjuster.txt');
    wireFrame = K * wireFrame';
    wireFrame(1,:) = wireFrame(1,:)./wireFrame(3,:);
    wireFrame(2,:) = wireFrame(2,:)./wireFrame(3,:);
    subplot(2,2,4)
    visualizeWireframe2D(img,wireFrame(1:2,:));
    title('After Shape Adjustment');
    suptitle(sprintf('Image %d',i));

    
    temp = 0;
    for j = 1:NumKeypoints
        if kps(3,j) > 0.2
        temp = temp + (wireFrame(1,j) - kps(1,j)) * (wireFrame(1,j) - kps(1,j)) + (wireFrame(2,j) - kps(2,j)) * (wireFrame(2,j) - kps(2,j));
        end
    end
    e = [e;temp];
    
    error = [error,e];
    
end
   
x = linspace(1,6,6);
figure, plot(x,error(1,:),'blue',x,error(2,:),'red',x,error(3,:),'green');
title('Reprojection Error at 3 Stages for each Image');
xlabel('Across 6 Images');
ylabel('Reprojection Error');
legend('Initial Error','Error after Pose Adjustment','Error after Shape Adjustment');
