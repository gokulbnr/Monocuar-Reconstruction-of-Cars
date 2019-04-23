function write_inp_shape(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanshape,vectors,theta2,kp_lookup)
pose = importdata('Ceres/ceres_output_singleViewPoseAdjuster.txt');

fID = fopen('Ceres/ceres_input_singleViewShapeAdjuster.txt','w');
fprintf(fID,'%d %d %d %d\n',1,36,36,size(vectors,1));
fprintf(fID,'%f %f %f\n',X3d(1),X3d(2),X3d(3));
fprintf(fID,'%f %f %f\n',avgCarHeight,avgCarWidth,avgCarLength);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',K(1,1),K(1,2),K(1,3),K(2,1),K(2,2),K(2,3),K(3,1),K(3,2),K(3,3));
for i=1:size(kps,2)
    fprintf(fID,'%f %f\n',kps(1,i),kps(2,i));
end
wV = kp_lookup(ceil(theta2),:)/(sum(kp_lookup(ceil(theta2),:)));
wHG = kps(3,:);
w = 0.5*(wV') + 0.5*wHG ;
for i=1:length(w)
    fprintf(fID,'%f\n',w(i));
end

%fprintf(fID,'%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n',0.125,0.000001,0.125,0.000001,0.000001,0.000001,0.125,0.125,0.125,0.000001,0.125,0.000001,0.125,0.125);

for i=1:size(meanshape,2)
    fprintf(fID,'%f %f %f\n',meanshape(1,i),meanshape(2,i),meanshape(3,i));
end
for i=1:size(vectors,1)
    for j=1:size(vectors,2)
        fprintf(fID,'%f',vectors(i,j));
        if j~=size(vectors,2) fprintf(fID,' ');end
    end
    fprintf(fID,'\n');
end

lambdas=[0.0208000000000000,0.00970000000000000,0.00720000000000000,0.00570000000000000,0.00470000000000000,0.00330000000000000,0.00210000000000000,0.00160000000000000,0.00100000000000000,0.000900000000000000,0.000800000000000000,0.000800000000000000,0.000700000000000000,0.000600000000000000,0.000500000000000000,0.000500000000000000,0.000400000000000000,0.000400000000000000,0.000400000000000000,0.000300000000000000,0.000300000000000000,0.000300000000000000,0.000300000000000000,0.000300000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000];
for j=1:size(vectors,1)
    fprintf(fID,'%f',lambdas(j));
    if j~=size(vectors,1) fprintf(fID,' ');end
end
fprintf(fID,'\n');

% fprintf(fID,'%f %f %f %f %f\n',0.25, 0.27 ,0.01, -0.08, -0.05);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',pose(1),pose(2) ,pose(3), pose(4),pose(5),pose(6),pose(7),pose(8),pose(9));
fprintf(fID,'%f %f %f\n',pose(10),pose(11) ,pose(12));

fclose(fID);

end
