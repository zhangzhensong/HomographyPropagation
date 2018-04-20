clear all;
    
work_dir = ['E:/democase/data/viewinterp/'];
imageApath = [work_dir '1/'];
imageBpath = [work_dir '2/'];
imageResult = [work_dir '3/'];

parfor k = 0:99
	disp(['k = ' num2str(k)]);
	image1 = mblend3([imageApath num2str(k) '.png'], [imageBpath num2str(k) '.png'], [imageApath num2str(k) '.hole.png'], [imageBpath num2str(k) '.hole.png'], k * 0.01);
	imwrite(image1, [imageResult num2str(k) '.png']);
end
    

