function dinp = Gfilt(inp)

Fcoef = fspecial('gaussian',5,0.5);
dinp = imfilter(inp,Fcoef,'conv');

return
























