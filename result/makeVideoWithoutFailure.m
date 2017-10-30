

close all 
clear
clc


% images = cell (10,1);
% images{1} = imread('CapturedFrames/frame_00040.png');
% images{2} = imread('CapturedFrames/frame_00041.png');
% images{3} = imread('CapturedFrames/frame_00042.png');
% images{4} = imread('CapturedFrames/frame_00043.png');
% images{5} = imread('CapturedFrames/frame_00044.png');
% images{6} = imread('CapturedFrames/frame_00045.png');
% images{7} = imread('CapturedFrames/frame_00046.png');
% images{8} = imread('CapturedFrames/frame_00047.png');
% images{9} = imread('CapturedFrames/frame_00048.png');
% images{10} = imread('CapturedFrames/frame_00049.png');
folderpath = 'CapturedFrames_withoutFailure';
images = dir(fullfile(folderpath,'*.png'));
images = {images.name}';

% create the video writer with 1 fps
 writerObj = VideoWriter('video_withoutFailure.avi');
 writerObj.FrameRate = 10;

% open the video writer
 open(writerObj);
 for ii = 1 : length(images)
      img = imread(fullfile(folderpath,images{ii}));
      writeVideo(writerObj,img)
 end


 % close the writer object
 close(writerObj);