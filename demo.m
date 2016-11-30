
% for visualization
addpath('/z/ywchao/codes/image-play/skeleton2d3d/H36M_utils/utils');
addpath('/z/ywchao/codes/image-play/skeleton2d3d/H36M_utils/external_utils');
addpath('/z/ywchao/codes/image-play/skeleton2d3d/H36M_utils/external_utils/lawrennd-mocap');

load('posSkel.mat');

% split = 'val';   sid = 147;  frid = 1;  ind = 1;
% split = 'val';   sid = 147;  frid = 53; ind = 53;
% split = 'test';  sid = 73;   frid = 1;  ind = 1894;

factor = 1.1;

% pred_dir = '/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-w1e-6/pred_val/';
% pred_dir = '/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-fts3-hg-w1e-6/pred_val/';
% pred_file = [pred_dir '00001.mat'];
% pred_dir = '/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-w1e-6/pred_test/';
% pred_file = [pred_dir '01894.mat'];
pred_dir = ['/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-w1e-6/pred_' split '/'];
pred_file = [pred_dir sprintf('%05d.mat',ind)];

pred = load(pred_file);

opt.seqLength = 16;

joints = [10,15,12,16,13,17,14,2,5,3,6,4,7];
repos = zeros(opt.seqLength,17,3);
repos(:,joints,:) = pred.repos;
repos(:,1,:) = (pred.repos(:,8,:) + pred.repos(:,9,:))/2;
repos(:,8,:) = (pred.repos(:,2,:) + pred.repos(:,3,:) + pred.repos(:,8,:) + pred.repos(:,9,:))/4;
repos(:,9,:) = (pred.repos(:,1,:) + pred.repos(:,2,:) + pred.repos(:,3,:))/3;
repos(:,11,:) = pred.repos(:,1,:);
repos = permute(repos,[1,3,2]);

% figure(1);
% j = 1;
% pred = permute(repos(j,:,:),[2 3 1]);
% V = pred;
% V([2 3],:) = V([3 2],:);
% showPose(V,posSkel);
% minx = -1000; maxx = 1000;
% miny = -1000; maxy = 1000;
% minz = -1000; maxz = 1000;
% axis([minx maxx miny maxy minz maxz]);
% set(gca,'ZTick',-1000:200:1000);
% set(gca,'ZDir','reverse');
% view([6,10]);

j = 1;
joints = [10 9 12 13 14 15 16 17 1 5 6 7 2 3 4];
skel = permute(repos(j,:,:),[2 3 1]);
skel = skel(:,joints);
skel = skel';
% convert to cmu mocap metric
skel = skel / 56.444;
% convert to cmu mocap coordinates
skel(:,2) = -skel(:,2);
skel(:,3) = -skel(:,3);

addpath('./prepare/mesh/skel/');
addpath('./prepare/mesh/quatern/');

skel_scape = load('./prepare/mesh/data/tpose.txt');
jointsRR = zeros(3, 3, 16, 1);

[RR,  R] = skel2RR(skel, skel_scape);
jointsRR(:, :, 1:15) = RR;
jointsRR(:, :, 16) = R;

% cal head
head = jointsRR(:, :, 3);
heads = matrix2quaternion(head);
% move head
matrix = q2matrix(heads);
matrix = matrix';
% valid
jointsRR(:, :, 3) = matrix*jointsRR(:, :, 3);

addpath('2-model/io');
addpath(genpath('./prepare/scape/MATLAB_daz_m_srf'));
Meta.instance.readA;
Meta.instance.readPCA;
% ponints weights
weights = Meta.instance.weight;
% textured models
load('./prepare/mesh3/data/arm_75_view_0');
% shape
% it is a 12 dim vector. You can set your own parameters
shapepara = Meta.instance.sem_default;
% RR2obj
skel_id = 1;
RR = jointsRR(:, :, 1:15, skel_id);
R = jointsRR(:, :, 16, skel_id);
% generate points
body = Body(RR, shapepara);
points = body.points;
% rot to original pose
p = R'*points';
% p = points';
p = 0.5*p;
% figure; scatter3(p(1,:),p(2,:),p(3,:));
% xlabel('x')
% ylabel('y')
% zlabel('z')
points = p';
points = moveToCenter(weights, points, 2);
p = points';

% BUG TAKE WASTES ME TWO DAYS:
%   Why is the behavior different for differnt filenames???
% obj
% fid = fopen('./test.obj', 'w');
fid = fopen('./001.obj', 'w');
% fid = fopen('./c.obj', 'w');
% fid = fopen('./d.obj', 'w');
% mtl
fprintf(fid, 'mtllib test.mtl\n');
% points
for i = 1:6449
    fprintf(fid, 'v %f %f %f\n', p(:, i));
end
% rest
fprintf(fid, '%s', restfiles);
fclose(fid);
% mtl
fid = fopen('./test.mtl', 'w');
% mtl
% fprintf(fid, 'newmtl Material\n');
% fprintf(fid, 'map_Kd  ../data/textures/1.png\n');
% fprintf(fid, 'newmtl Material2\n');
% fprintf(fid, 'map_Kd  ../data/textures2/1.png\n');
fprintf(fid, 'newmtl Material\n');
fprintf(fid, 'map_Kd  ./prepare/mesh3/data/1.png\n');
fprintf(fid, 'newmtl Material2\n');
fprintf(fid, 'map_Kd  ./prepare/mesh3/data/1.png\n');
fclose(fid);

addpath('../4-render');
% % [p, ~] = get_data_from_obj_blender('test.obj');
% [p, ~] = get_data_from_obj_blender('001.obj');
% r = 85/180*3.1416;
% rr = [1, 0, 0; 0, cos(r), -sin(r);0, sin(r), cos(r)];
% p = rr'*p;
% % skeleton
% pp = p';
% [weights_sort, ind] = sort(weights, 2);
% skel = points2skel(pp, weights_sort, ind);
% % wangchunyu skeleton
% skelton = skel([4, 1, 17, 18, 19, 13, 14, 15, 3, 9, 10, 11, 5, 6, 7], :);
% % [pre, name, ~] = fileparts('./test.obj');
% [pre, name, ~] = fileparts('./001.obj');
% fid = fopen([pre '/' name '.txt'], 'w');
% for k = 1:15
%     fprintf(fid, '%f %f %f\n', skelton(k, :));
% end
% fclose(fid);
% % conver to h36m format
% joints = [10 9 12 13 14 15 16 17 1 5 6 7 2 3 4];
% S = zeros(17,3);
% S(joints,:) = skelton;
% S(8,:) = (skelton(2,:) + skelton(9,:))/2;
% S(11,:) = skelton(1,:);
% % convert to mm
% S = S * 56.444;
% % subtract mean
% S = S - repmat(mean(S,1),[17 1]);
% % convert to camera coordinates
% S(:,2) = -S(:,2);
% S(:,3) = -S(:,3);
% % display skeleton
% V = S';
% V([2 3],:) = V([3 2],:);
% hpos = showPose(V,posSkel);
% minx = -40; maxx = 40;
% miny = -40; maxy = 40;
% minz = -40; maxz = 40;
% axis([minx maxx miny maxy minz maxz]);
% set(gca,'ZTick',-1000:200:1000);
% set(gca,'ZDir','reverse');
% view([6,10]);

% a = -90+20;  % 0;
% b = -90+20;  % 0;
% c = 0+20;    % 0;
% d = 2.15;
a = 0;   % -90+20;  % 0;
b = 90;  % 0+20;    % 0;
c = -90; % 0;
d = 2.15;
% blender path
g_blender_path = '/z/ywchao/tools/blender-2.78a-linux-glibc211-x86_64/blender';
% blank file
blankFile = '/z/ywchao/codes/image-play/Deep3DPose/4-render/blank.blend';
% render file
renderFile = '/z/ywchao/codes/image-play/Deep3DPose/4-render/render_model_views_ip.py';
% obj file
% objFile = 'test.obj';
objFile = '001.obj';
% objFile = 'c.obj';
% objFile = 'd.obj';
% render folder
renderFolder = '.';
% view file
viewname = 'view.txt';
fp = fopen(viewname, 'w');
fprintf(fp, '%f %f %f %f\n', a,b,c,d);
fclose(fp);
% render
command = sprintf('%s %s --background --python %s -- %s %s %s',...
        g_blender_path, blankFile, renderFile, objFile, renderFolder, viewname);
system(command);

% % 3d skeleton name
% % data3d = load('test.txt');
% data3d = load('001.txt');
% % camera set path
% % campath = 'image_a-70_e-70_t020_d002_cam.txt';
% campath = 'image_a-70_e020_t000_d002_cam.txt';
% camdata = load(campath);
% c = camdata(1, 1:3);
% q = camdata(2, :);
% data3d2 = blenmder_points(data3d, c, q, 15);
% near = 0.01;
% far = 4;
% top = near*tan(28.8418/2/180*pi);
% right = top/540*960;
% promatr = [near/right, 0, 0, 0;0, near/top, 0, 0;...
%     0, 0, -(far+near)/(far-near), -2*far*near/(far-near);...
%     0, 0, -1, 0];
% data2d3 = promatr*[data3d2; ones(1, 15)];
% data2d4 = data2d3./repmat(data2d3(3, :), 4, 1);
% data2d4 = data2d4(1:2, :);
% data2d4 = (data2d4+1)/2;
% data2d = data2d4';
% imgpath = campath(1:end-8);
% tmoimpath = [imgpath '.png'];
% [tmpim, ~, tmpalpha] = imread([imgpath '.png']);
% tmpim = double(tmpim)/255;
% tmpalpha = double(tmpalpha)/255;
% % crop image
% [tmpalpha2, tmpim2, data2d2] = crop(tmpalpha, 0, tmpim, data2d);
% % background
% im_backname = '/z/ywchao/codes/image-play/data/Penn_Action_cropped/frames/0147/000001.jpg';
% im_back = imread(im_backname);
% re_im = combine(tmpalpha2, tmpim2, im_back);
% 
% imshow(re_im)

% load rendered image
[im_fg, ~, alpha_fg] = imread('image_a000_e090_t-90_d002.png');
im_fg = double(im_fg);
alpha_fg = double(alpha_fg);

% crop rendered image
bgColor = 0;
[nr, nc] = size(alpha_fg);
colsum = sum(alpha_fg == bgColor, 1) ~= nr;
rowsum = sum(alpha_fg == bgColor, 2) ~= nc;
ll = find(colsum, 1, 'first');
rr = find(colsum, 1, 'last');
tt = find(rowsum, 1, 'first');
bb = find(rowsum, 1, 'last');
alpha_fg = alpha_fg(tt:bb, ll:rr);
im_fg = im_fg(tt:bb, ll:rr, :);

% load background image
% im_bg_name = '/z/ywchao/codes/image-play/data/Penn_Action_cropped/frames/0147/000001.jpg';
% im_bg_name = '/z/ywchao/codes/image-play/data/Penn_Action_cropped/frames/0073/000001.jpg';
im_bg_name = sprintf('/z/ywchao/codes/image-play/data/Penn_Action_cropped/frames/%04d/%06d.jpg',sid,frid);
im_bg = imread(im_bg_name);
im_bg = double(im_bg);

% load 2d prediction
joints = [10,15,12,16,13,17,14,2,5,3,6,4,7];
% eval_dir = '/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-w1e-6/eval_val/';
% eval_file = [eval_dir '00001.mat'];
% eval_dir = '/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-w1e-6/eval_test/';
% eval_file = [eval_dir '01894.mat'];
eval_dir = ['/z/ywchao/codes/image-play/exp/penn-crop/seq16-hg-256-res-clstm-res-64-w1e-6/eval_' split '/'];
eval_file = [eval_dir sprintf('%05d.mat',ind)];
preds = load(eval_file);
pred2 = zeros(opt.seqLength,17,2);
pred2(:,joints,:) = preds.eval;
pred2(:,1,:) = (preds.eval(:,8,:) + preds.eval(:,9,:))/2;
pred2(:,8,:) = (preds.eval(:,2,:) + preds.eval(:,3,:) + preds.eval(:,8,:) + preds.eval(:,9,:))/4;
pred2(:,9,:) = (preds.eval(:,1,:) + preds.eval(:,2,:) + preds.eval(:,3,:))/3;
pred2(:,11,:) = preds.eval(:,1,:);

% factor = 1.2;
% factor = 1.0;
skel2 = permute(pred2(j,:,:),[2 3 1]);
x1 = min(skel2(:,1));
x2 = max(skel2(:,1));
y1 = min(skel2(:,2));
y2 = max(skel2(:,2));
h = (y2-y1) * (factor-1);
w = (x2-x1) * (factor-1);
x1 = round(x1 - w/2);
y1 = round(y1 - h/2);
x2 = round(x2 + w/2);
y2 = round(y2 + h/2);
x1 = max(x1,1);
y1 = max(y1,1);
x2 = min(x2,size(im_bg,2));
y2 = min(y2,size(im_bg,1));

% imshow(uint8(im_bg)); hold on;
% rectangle('Position',[x1,y1,x2-x1+1,y2-y1+1],'EdgeColor','g');

% resize foreground image
im_fg = imresize(im_fg,[y2-y1+1,x2-x1+1]);
alpha_fg = imresize(alpha_fg,[y2-y1+1,x2-x1+1]);
alpha_fg = min(alpha_fg,1);
alpha_bg = 1 - alpha_fg;

im_cb = uint8(im_bg);
for k = 1:3
    fg = uint8(alpha_fg .* im_fg(:,:,k));
    bg = uint8(alpha_bg .* im_bg(y1:y2,x1:x2,k));
    im_cb(y1:y2,x1:x2,k) = fg + bg;
end

figure(1); imshow(im_cb);
