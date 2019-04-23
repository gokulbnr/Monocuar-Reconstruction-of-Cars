clc; clear all; close all;

innputVal = importdata('infofile.txt');

iter = 0;
flag = -1;
count = 0;

for iterations=1:size(innputVal,1)   
    if iterations == size(innputVal,1)
        las = iterations;       
        code2
        clearvars -except iterations innputVal carID seq subseq flag fir las iter;
    end
    if innputVal(iterations,1) ~= flag        
        if flag == -1 && iterations == 1
            fir = 1;
            carID = innputVal(iterations,4);
            seq = innputVal(iterations,2);
            subseq = innputVal(iterations,1);
            flag = innputVal(iterations,1);
        else
            las = iterations-1;     
            code2
            clearvars -except iterations innputVal carID seq subseq flag fir las iter
            flag = innputVal(iterations,1);
            fir = iterations;
            carID = innputVal(iterations,4);
            seq = innputVal(iterations,2);
            subseq = innputVal(iterations,1);
        end      
    end   
end
