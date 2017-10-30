% https://www.mathworks.com/help/matlab/examples/convert-between-image-sequences-and-video.html

close all 
clear
clc

frame_speed = 10;
folderpath = 'CapturedFrames_withFailure';
images = dir(fullfile(folderpath,'*.png'));
images = {images.name}';

% create the video writer with 1 fps
 writerObj = VideoWriter('video_withFailure.avi');
 writerObj.FrameRate = frame_speed;

% open the video writer
 open(writerObj);
 for ii = 1 : length(images)
      img = imread(fullfile(folderpath,images{ii}));
      writeVideo(writerObj,img)
 end


 % close the writer object
 close(writerObj);