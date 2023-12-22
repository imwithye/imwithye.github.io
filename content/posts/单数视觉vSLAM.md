---
title: "单目视觉vSLAM"
date: 2023-12-22
tags: ["Computer Vision", "SLAM", "MATLAB", "算法"]
---

Matlab 官方提供了完整的单目视觉 vSLAM 的 pipeline，[https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html](https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html)。这里对 Matlab 的这边文章的关键点做一个笔记和讨论，具体的实现可以参考原文文档。

## 初始化数据集

Matlab 提供了`imageDatastore`类用于初始化图像存储集合，其接受一个图像的文件夹路径的参数。

```matlab
imageFolder   = [dataFolder,'rgbd_dataset_freiburg3_long_office_household/rgb/'];
imds          = imageDatastore(imageFolder);
```

## 初始化地图

在 SLAM 管线中，首先我们应该对相机进行标定，相机标定可以用[Computer Vision 工具箱](https://www.mathworks.com/products/computer-vision.html)中的相机标定工具。如果预先知道了相机的内参，可以通过`cameraIntrinsics`类直接进行初始化。

```matlab
% Create a cameraIntrinsics object to store the camera intrinsic parameters.
% The intrinsics for the dataset can be found at the following page:
% https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
% Note that the images in the dataset are already undistorted, hence there
% is no need to specify the distortion coefficients.
focalLength    = [535.4, 539.2];     % in units of pixels
principalPoint = [320.1, 247.6];     % in units of pixels
imageSize      = size(currI,[1 2]);  % in units of pixels
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);
```

在例子中使用的数据集已经提供了相机的标定参数，我们可以直接使用。注意，对于一般相机获取到的照片来说，我们应该进行照片的去畸变，Matlab 提供了去畸变函数`undistortImage`，具体用法如下

```matlab
% Get intrinsic parameters of the camera
intrinsics = cameraParams.Intrinsics;
I = undistortImage(I, intrinsics);
```

### 识别并抓取图片特征

在该例子中，我们使用 ORB 特征，ORB 特征是一种广泛使用的特征，并且能够快速识别，更好的用于实时 SLAM 环境。借助`helperDetectAndExtractFeatures`函数，我们可以识别出图片中的特征和其对应的位置。

```matlab
% Detect and extract ORB features
scaleFactor = 1.2;
numLevels   = 8;
numPoints   = 1000;
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints);
```

进一步的，我们可以可视化图片的特征

```matlab
I = readimage(imds, 1);
imshow(I);
hold on;
plot(selectStrongest(prePoints, 100));
```

抓取特征以后，下一步进行特征匹配，由于我们的图片是从视频流中获得的有序图片，我们保留第一张图片，并且从第二张图片开始，与第一张图片做特征匹配。当特征少于 100 个时，我们选择下一张图片与第一张图片匹配，直到找到对应的匹配图片为止。

```matlab
firstI       = currI;              % Preserve the first frame
currFrameIdx = currFrameIdx + 1;   % set current to 2
while ~isMapInitialized && currFrameIdx < numel(imds.Files)
    currI = readimage(imds, currFrameIdx);
    currFrameIdx = currFrameIdx + 1;

    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, ...
        scaleFactor, numLevels, numPoints);
    indexPairs = matchFeatures(preFeatures, currFeatures, Unique=true, ...
        MaxRatio=0.9, MatchThreshold=40);

    minMatches = 100;
    if size(indexPairs, 1) < minMatches; continue; end
    % ...
```

找到对应的图片以后，我们利用对极约束求解`Homography`和基础矩阵`Fundamental matrix`。当场景为平面时，`Homography`比基础矩阵更容易匹配。

```matlab
% Select the model based on a heuristic
ratio = scoreH/(scoreH + scoreF);
ratioThreshold = 0.45;
if ratio > ratioThreshold
    inlierTformIdx = inliersIdxH;
    tform          = tformH;
else
    inlierTformIdx = inliersIdxF;
    tform          = tformF;
end
```

我们根据匹配的结果选择对应的`Homography`或者`Fundamental matrix`。这里`inlierTformIdx`表示用于求解的特征点的下标，`tfrom`则是对应的变换矩阵。

### 相对位姿估计

在得到对应的变换矩阵的求解点（inlier）以后，我们可以进行相对位姿估计。我们用一半的特征点进行估计，用来减少估计的成本。

```matlab
% Computes the camera location up to scale. Use half of the
% points to reduce computation
inlierPrePoints  = preMatchedPoints(inlierTformIdx);
inlierCurrPoints = currMatchedPoints(inlierTformIdx);
[relPose, validFraction] = estrelpose(tform, intrinsics, ...
    inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));

% If not enough inliers are found, move to the next frame
if validFraction < 0.9 || numel(relPose)==3
    continue
end
```

如果`validFraction`太小，一般来说小于 0.9，那就意味着基础矩阵是不正确的。或者如果我们的相对位姿计算不正确，那我们就应该跳过该图片，利用下一张图片求解。

### 三角化和稀疏点云估计

当我们获得了相对位姿以后，我们可以利用三角化求解稀疏点云。当我们的两张图片过于靠近时，可能会出现退化问题，于是我们将退化的位姿过滤。

```matlab
% Triangulate two views to obtain 3-D map points
minParallax = 1; % In degrees
[isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
    rigidtform3d, relPose, inlierPrePoints, inlierCurrPoints, intrinsics, minParallax);

if ~isValid
    continue
end
```

### 初始化结果

至此，我们完成了整个的地图的初始化工作，我们可以查看对应的图片和初始化效果。根据运行的结果，我们可以看到最后使用了第 1 张和第 29 张图片完成了地图的初始化工作。初始化的特征匹配结果如下

![](/img-posts/单目视觉vSLAM_1.png)

## 建立关键帧和地图空间点存储集合

Matlab 提供了两个类来存储关键帧和地图空间点，分别是`imageviewset`和`worldpointset`。我们创建关键帧`viewset`以后，将用于地图初始化的两张图片作为初始的俩个关键帧存入，同时我们加入关键帧相关性链接。相关性指的是俩个关键帧之间的相对位姿，以及相关的匹配点。最后我们将三角化后的世界坐标加入到地图的世界点集合中。

```matlab
vSetKeyFrames = imageviewset;
mapPointSet   = worldpointset;

preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigidtform3d, Points=prePoints,...
    Features=preFeatures.Features);
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, Points=currPoints,...
    Features=currFeatures.Features);

vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, Matches=indexPairs);
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, xyzWorldPoints);
```

这里`newPointIdx`是添加的世界点的下标集合。每一个世界点都是有两帧关键帧的匹配点通过对极几何约束和三角化处理得到，因此在`indexPairs`中的第一组点对应的第一关键帧中的点序列，第二组点对应的是第二个关键帧中的点序列。在`mapPointSet`中，我们需要将这一组对应关系添加进去，之后的后端优化中，我们将使用到世界点和像点的对应关系。

```matlab
mapPointSet   = addCorrespondences(mapPointSet, preViewId, newPointIdx, indexPairs(:,1));
mapPointSet   = addCorrespondences(mapPointSet, currViewId, newPointIdx, indexPairs(:,2));
```

## 回环检测和初始化地标识别数据库

我们初始化地标识别数据库用于回环检测算法。地标识别使用的是词袋模型。我们可以从一个大数据采集的图像集合中生成字典，生成的方法为

```matlab
bofData = bagOfFeatures(imds, ...
    CustomExtractor=@helperORBFeatureExtractorFunction, ...
    TreeProperties=[3, 10], StrongestFeatures=1);
```

在这里我们使用一个预生成的字典文件，并且生成回环检测数据库

```matlab
bofData      = load("bagOfFeaturesDataSLAM.mat");
loopDatabase = invertedImageIndex(bofData.bof,SaveFeatureLocations=false);
```

接下来，我们将第一帧和第二帧的特征数据以及`viewId`添加到数据库。

```matlab
addImageFeatures(loopDatabase, preFeatures, preViewId);
addImageFeatures(loopDatabase, currFeatures, currViewId);
```

## 第一次后端优化和重建可视化

当我们有了世界点的坐标，位姿以及成像坐标以后，我们可以使用`bundleAdjustment`对数据进行优化，寻找最佳的位姿以达到最少的重投影误差。Matlab 内建了`bundleAdjustment`函数，我们需要输入世界点的坐标，视图中相关联的对应点`tracks`，相机位姿信息，相机内参。同时，我们可以指定哪些关键帧的位姿是不需要调整的，在这里我们第一帧的位置将会被固定。我们第一次后端优化仅仅使用了俩个关键帧，在双视图几何中，俩个相对应的像点，形成一个世界空间的三位点，因此`xyzWorldPoints`的大小应该和`tracks`的大小一致，并且每个`tracks`中的点对，成像一个`xyzWorldPoints`中的点。

```matlab
tracks       = findTracks(vSetKeyFrames);
cameraPoses  = poses(vSetKeyFrames);

[refinedPoints, refinedAbsPoses] = bundleAdjustment(xyzWorldPoints, tracks, ...
    cameraPoses, intrinsics, FixedViewIDs=1, ...
    PointsUndistorted=true, AbsoluteTolerance=1e-7,...
    RelativeTolerance=1e-15, MaxIteration=20, ...
    Solver="preconditioned-conjugate-gradient");
```

在`bundleAdjustment`之后，我们得到了调整后的点以及调整后的相机位姿态。我们应该将调整后的点重新应用到我们的关键帧中。为了可视化方便，我们将坐标点和位姿的都按统一尺度进行缩放

```matlab
% Scale the map and the camera pose using the median depth of map points
medianDepth   = median(vecnorm(refinedPoints.'));
refinedPoints = refinedPoints / medianDepth;

refinedAbsPoses.AbsolutePose(currViewId).Translation = ...
    refinedAbsPoses.AbsolutePose(currViewId).Translation / medianDepth;
relPose.Translation = relPose.Translation/medianDepth;

% Update key frames with the refined poses
vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);
vSetKeyFrames = updateConnection(vSetKeyFrames, preViewId, currViewId, relPose);

% Update map points with the refined positions
mapPointSet = updateWorldPoints(mapPointSet, newPointIdx, refinedPoints);

% Update view direction and depth
mapPointSet = updateLimitsAndDirection(mapPointSet, newPointIdx, vSetKeyFrames.Views);

% Update representative view
mapPointSet = updateRepresentativeView(mapPointSet, newPointIdx, vSetKeyFrames.Views);
mapPlot = helperVisualizeMotionAndStructure(vSetKeyFrames, mapPointSet);
```

最后我们将位姿和世界点绘图得到

![](/img-posts/单目视觉vSLAM_2.png)

## SLAM 位姿跟踪

现在我们开始实现 SLAM 的位姿跟踪。每当我们发现了一个新的关键帧，我们就将其加入到我们的数据集中，并计算和调整我们的位姿态，为了简化我们的跟踪问题，当我们检测到回环的时候，我们停止跟踪。