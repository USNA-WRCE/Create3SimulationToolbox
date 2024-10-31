function ps = bw2polyshape(bin,sc,H_c2o)
% BW2POLYSHAPE converts a binary image with associated scaling to a patch
% object with simulated height
%   ptch = bw2polyshape(bin,sc,H_c2o)
%   ptch = bw2polyshape(bin,sc)
%
%   Input(s)
%       bin - MxN binary image (black/false represents obstacles)
%        sc - scalar value defining the ratio of linear units per pixel. 
%     H_c2o - 4x4 transformation defining the "upper left corner" frame of
%             free-space relative to the desired world/global frame for the
%             returned polyshape. Default value if unspecified is the
%             identity.
%
%   Output(s)
%       ptch - structured array defining "Faces" and "Vertices" of a patch
%              object.
%
%   Example(s)
%       % Create binary image
%       im = imread('Occupancy208_75m2p.png');
%       bin = imbinarize( rgb2gray(im) );
%
%       % Define patch information
%       H_c2o = Tx(3098.8/1000)*Ty(-5384.8/1000)*Rz(pi/2);
%       pix2m = 1/75;
%       ps = bw2polyshape(bin,pix2m,H_c2o);
%
%       % Plot patch
%       fig = figure('Name','bw2polyshape Example');
%       axs = axes('Parent',fig,'NextPlot','add',...
%           'DataAspectRatio',[1 1 1],'ZDir','Reverse','YDir','Reverse');
%       view(axs,3);
%       lgt = addSingleLight(axs);
%       set(lgt,'Position',[1,0,-1]);
%       plt = plot(ps,'FaceColor','k');
%       sc = min( abs(ps.boundingbox) );
%       h_o2a = triad('Parent',axs,'Scale',sc/2,'LineWidth',1.5,...
%           'AxisLabels',{'g_1','g_2','g_3'});
%
%   M. Kutzer, 15Oct2024, USNA

debug = false;

%% Check input(s)
narginchk(2,3);

if ~ismatrix(bin) || ~islogical(bin)
    error('Binary image must be defined using an MxN logical array.');
end

if numel(sc) ~= 1 || ~isnumeric(sc)
    error('Scaling factor must be defined as a scalar numeric value.');
end

if nargin < 3
    H_c2o = eye(4);
end

%% Setup debug plot
if debug
    fig = figure('Name','bw2polyshape.m, debug = true');
    axs = axes('Parent',fig,'NextPlot','add','DataAspectRatio',[1 1 1]);
    img = imshow(bin,'Parent',axs);
    set(axs,'Visible','on');
    xlabel(axs,'x (pixels)');
    ylabel(axs,'y (pixels)');
end

%% Process binary image
% Label segmented objects
[lbl_t,n_t] = bwlabel( bin);
[lbl_f,n_f] = bwlabel(~bin);

% Find perimeters
i = 0;
% TRUE
for j = 1:n_t
    i = i+1;
    % Select ith segmented object
    bw = lbl_t == j;
    % Choose a point on the object
    [r,c] = find(bw,1,'first');
    % Define the contour of the object
    contour{i} = fliplr( bwtraceboundary(bw,[r,c],'N') );
    isFree(i) = true;
end
% FALSE
for j = 1:n_f
    i = i+1;
    % Select ith segmented object
    bw = lbl_f == j;
    % Choose a point on the object
    [r,c] = find(bw,1,'first');
    % Define the contour of the object
    contour{i} = fliplr( bwtraceboundary(bw,[r,c],'N') );
    isFree(i) = false;
end

%% Remove contours containing less than 3 points
tfRemove = cellfun(@(x)(size(x,1) < 3),contour);
contour(tfRemove) = [];
isFree(tfRemove) = [];

%% Check for empty contour
if numel(contour) == 0
    ps = polyshape;
    return
end

%% Convert contours to polyshapes
warning('off','MATLAB:polyshape:repairedBySimplify');
for i = 1:numel(contour)
    psTMP(i) = polyshape(contour{i}(:,1),contour{i}(:,2));
    areaTMP(i) = area(psTMP(i));
end
warning('on','MATLAB:polyshape:repairedBySimplify');

%% Sort areas
[areaTMP,idx] = sort(areaTMP,2,'descend');
psTMP = psTMP(idx);
isFree = isFree(idx);

%% Remove bounding "free" region(s)
tfRemove = false(1:numel(psTMP));
for i = 1:numel(psTMP)
    if isFree(i)
        tfRemove(i) = true;
    else
        break
    end
end
psTMP(tfRemove) = [];
areaTMP(tfRemove) = [];
isFree(tfRemove) = [];

%% Define common polyshape
ps = psTMP(1);
for i = 2:numel(psTMP)
    if isFree(i)
        ps = subtract(ps,psTMP(i));
    else
        ps = union(ps,psTMP(i));
    end
end

if debug
    plt_ps = plot(axs,ps,'FaceColor','g','EdgeColor','None','FaceAlpha',0.5);
end

%% Crop excessive boundary
%{
psH = ps.holes;
Xh = psH.Vertices.';
Xp = ps.Vertices.';
for i = 1:size(Xh,1)
    Xp_lims(i,:) = [min(Xp(i,:)),max(Xp(i,:))];
    Xh_lims(i,:) = [min(Xh(i,:)),max(Xh(i,:))];
end

% Define inner and outer boarder
Xout = [...
    Xp_lims(1,1), Xp_lims(1,2), Xp_lims(1,2), Xp_lims(1,1);...
    Xp_lims(2,1), Xp_lims(2,1), Xp_lims(2,2), Xp_lims(2,2)];
Xin = [...
    Xh_lims(1,1), Xh_lims(1,2), Xh_lims(1,2), Xh_lims(1,1);...
    Xh_lims(2,1), Xh_lims(2,1), Xh_lims(2,2), Xh_lims(2,2)];

% Add buffer
buffer = 5;
Xout = Xout + buffer*[-1,1,1,-1;-1,-1,1,1];
Xin = Xin   + buffer*[-1,1,1,-1;-1,-1,1,1];

psOut = polyshape(Xout(1,:),Xout(2,:));
psIn = polyshape(Xin(1,:),Xin(2,:));

psBoarder = subtract(psOut,psIn);

ps = subtract(ps,psBoarder);
%}

%% Scale vertices
Xp = ps.Vertices.';
Xp(4,:) = 1;

A_p2v = Sx(sc)*Sy(sc);

Xv = A_p2v*Xp;

Xv = Xv(1:2,:);

ps.Vertices = Xv.';

%% Reference to "upper left corner" of free space bounding box
psH = ps.holes;

idx0 = psH.nearestvertex(0,0);
Xv0 = psH.Vertices(idx0,:).';

Xc = Xv - repmat(Xv0,1,size(Xv,2));

ps.Vertices = Xc.';

%% Reference to global/world frame
Xc(4,:) = 1;

Xo = H_c2o*Xc;

ps.Vertices = Xo(1:2,:).';

%% Debug plot
if debug
      % Plot patch
      fig = figure('Name','bw2polyshape, debug = true');
      axs = axes('Parent',fig,'NextPlot','add',...
          'DataAspectRatio',[1 1 1],'ZDir','Reverse','YDir','Reverse');
      view(axs,3);
      lgt = addSingleLight(axs);
      set(lgt,'Position',[1,0,-1]);
      plt = plot(ps,'FaceColor','k');
      sc = min( abs(ps.boundingbox) );
      h_o2a = triad('Parent',axs,'Scale',sc/2,'LineWidth',1.5,...
          'AxisLabels',{'g_1','g_2','g_3'});
      h_c2o = triad('Parent',h_o2a,'Matrix',H_c2o,'Scale',sc/3,...
          'LineWidth',1.5,'AxisLabels',{'c_1','c_2','c_3'});
end