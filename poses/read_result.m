%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
%Getting the image 
dirload = 'D:\Action_recognition_skeleton\Du_lieu2_14\poses\color_l10';
dirloadsave = 'D:\Action_recognition_skeleton\Du_lieu2_14\poses\color_l10\data_edit';


files = dir(strcat(dirload,'\*.txt'));
numFiles = size(files,1);
filesName = cell(numFiles,1);
for i = 1:numFiles 
        filename= strcat(dirload,'\',files(i).name);
        fileID = fopen(filename,'r');
        %                                           1               2   
        formatSpec = '%d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
        A = fscanf(fileID,formatSpec);
        fclose(fileID);
        kt=size(A,1);
         if (kt<=78)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
             filename_new= strcat(dirloadsave,'\',files(i).name);
             fileID_new = fopen(filename_new,'w');

           %%%%%%%%%%%%%%%%%%%%%%%%%%%
             j=4;
            label=1;

            while (j<=kt)
                x=round(A(j,1));
                y=round(A(j+1,1));
                IOU= A(j+2,1);
                fprintf(fileID_new,'%d %d %d %f\n',label,x,y,IOU);
                label=label+1;
                j= j+3;
            end
             fclose(fileID_new);
      end
end
