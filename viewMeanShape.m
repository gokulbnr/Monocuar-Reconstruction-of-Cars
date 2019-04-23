% clc; clear all; close all;

%Initialization of meanshape and deformation vectors
meanShape = importdata('meanShape.txt');
vectors = importdata('vectors.txt');
meanShape = meanShape';

%Apply Scaling on meanShape and basis vectors
%Mean Dimensions
avg = [ 3.8600 ; 1.5208 ; 1.6362 ];
plt = [max(meanShape(1,:))-min(meanShape(1,:)) ; %length 
       max(meanShape(2,:))-min(meanShape(2,:)) ; %height
       max(meanShape(3,:))-min(meanShape(3,:))]; %width
scale_factor = avg./plt;
meanShape = diag(scale_factor) * meanShape;
for i=1:42
    mat = reshape(vectors(i,:),3,36);
    sca = diag(scale_factor)*mat;
    vectors(i,:) = reshape(mat,1,108);
end

%Apply Rotations
rot_mat =  roty(90) * rotz(180);

meanShape = rot_mat * meanShape;
for i=1:42
    mat = reshape(vectors(i,:),3,36);
    mat = rot_mat * mat;
    vectors(i,:) = reshape(mat,1,108);
end

%Send the 3X36 meanShape Matrix to the given function to get the plot
visualizeWireframe3D(meanShape);