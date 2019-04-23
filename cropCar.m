function res = cropCar(seq,id,fir,las)
    root_dir = '/home/gokulbnr/RRC/IROS18_Multi';
    image_dir = '/media/gokulbnr/GBN - My Passport/RRC/Datasets/KITTI/data_tracking_image_2/training/image_02';
    label_dir = '/media/gokulbnr/GBN - My Passport/RRC/Datasets/KITTI/data_tracking_label_2/training/label_02';
    info_dir = '/home/gokulbnr/RRC/IROS18_Multi/infoFiles';
    infofile = sprintf('%s/info%d.txt',info_dir,seq);
    fid = fopen(infofile,'w');
    
    tracklets = readLabels(label_dir,seq);
    for frm = fir:las
        filename = sprintf('%s/%04d/%06d.png',image_dir,seq,frm);
        img = imread(filename);
%         figure, imshow(img); hold on;
        node = tracklets{frm+1};
        for obj = 1:size(node,2)
            if(strcmp(node(obj).type,'Car') == 1) && (node(obj).id == id)
                fprintf(fid,'%d %d\n',node(obj).frame,node(obj).id);
%                 rectangle('Position',[node(obj).x1;node(obj).y1;node(obj).x2-node(obj).x1;node(obj).y2-node(obj).y1],'EdgeColor','red','LineWidth',2);
%                 text(node(obj).x1,node(obj).y1,sprintf('%d',node(obj).id),'BackgroundColor','black','Color','white');
                f = figure('visible','off');
                im = imcrop(img,[node(obj).x1;node(obj).y1;node(obj).x2-node(obj).x1;node(obj).y2-node(obj).y1]);
                imshow(im);
                imwrite(im,['crop/' num2str(frm) '.jpg'])

%                 saveas(im,['crop/' num2str(frm) '_' num2str(id) '.jpg'])
            end
        end
         pause(1);
         close all;
    end
    fclose(fid);
    res = 1;
end