function [im] = mblend3(image_name1, image_name2, holemask_name1, holemask_name2, alpha)

im1 = imread(image_name1);
im2 = imread(image_name2);

mask1 = imread(holemask_name1);
mask2 = imread(holemask_name2);

mask1 = mask1 < 128;
mask2 = mask2 < 128;

% r = 10;
% mask1 = dilate(mask1, r);
% mask2 = dilate(mask2, r);

im1(mask1) = im2(mask1);
im2(mask2) = im1(mask2);

im1 = im2double(im1);
im2 = im2double(im2);

im1p{1} = im1;
im2p{1} = im2;

mp1{1} = im2double(mask1);
mp2{1} = im2double(mask2);

M = 8;

for n = 2 : M
    im1p{n} = imresize(im1p{n-1}, 0.5);
    im2p{n} = imresize(im2p{n-1}, 0.5);

    mp1{n} = imresize(mp1{n-1}, 0.5, 'bilinear');
    mp2{n} = imresize(mp2{n-1}, 0.5, 'bilinear');
end

for n = 1 : M-1
    im1p{n} = im1p{n} - imresize(im1p{n+1}, [size(im1p{n},1), size(im1p{n},2)]);
    im2p{n} = im2p{n} - imresize(im2p{n+1}, [size(im2p{n},1), size(im2p{n},2)]);   
end   

for n = 1 : M
    imp{n} = alpha * im2p{n} + (1 - alpha) * im1p{n};
end

im = imp{M};
for n = M-1 : -1 : 1
    im = imp{n} + imresize(im, [size(imp{n},1) size(imp{n},2)]);
end
