## 基于平面的点云配准方法实际测试

测试论文：Point cloud registration and localization based on voxel plane features, ISPRS 2022.

### 官方的demo数据和默认参数

+ 没有真值，目视检查。
+ 首先使用VPFB配准两份点云，将结果与直接使用CloudCompare的Trimm-ICP配准的结果进行比较。

#### 可视化图

+ 可以发现效果是非常不错的，用来做SLAM效果应该也挺好的。
+ 下图第一行：原始点云对、使用VPFB的配准结果、CC的T-ICP的配准结果。第二行和第三行是细节对应放大图。

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/demo_total.png)

#### 简单分析

明显可以发现T-icp，90%重叠下的结果中有**明显错乱**，而VPFB表现就非常好了。(点云中存在的非刚性畸变是另一个问题了)。**如果在VPFB-R之后再接一个T-ICP的方法（90%重叠度），可以发现效果会好非常多，那么整体就是coarse-to-fine的方案，也能保证结果不错**。而且速度挺快。

### 单线稠密MLS点云数据测试

+ 红色为初始配准之后的结果，可以发现由于场景中平面对数量不足，导致匹配结果容易出现不稳定的情况，对场景比较依赖。
+ 简单调整参数之后，选择平面数量越多可以保证结果比较好，其他参数没什么变化。

![image-20230711123258963](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/image-20230711123258963.png)

#### 平面检测

+ 尝试去掉地面，发现地面效果去除的还挺好的，但是VPFB的配准效果也不怎么样。

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/Section0_non_ground.png)

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/Section61_non_ground.png)

+ 重新调整了一个k1（1->2）参数，发现效果还可以的。而这个和平面有无关系不大了。

### TLS&MLS点云配准

+ 在跨源点云匹配中发现参数又不行了。

#### 可视化图

TODO

## BSC点云配准方法实际测试

测试论文：A novel binary shape context for 3D local surface description， ISPRS 2017.

### TLS&MLS点云配准

+ 真值用CloudCompare手动标记。
+ 平台I7-9700K,3.6GHz。TLS和MLS点云都比较大，因此会稍微耗时一些。

#### 可视化图

+ 由于MLS本身存在分层，因此其实最终的结果并不是很好，但是比原始状态好太多了。

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/bsc_tls&mls_all_in_one.png)

#### 定量的指标

|   不同实现方式    | 关键点数量TLS-MLS | 匹配对 | 特征提取耗时s(有IO) | 匹配耗时s(无IO) |                 精度                  |
| :---------------: | :---------------: | :----: | :-----------------: | :-------------: | :-----------------------------------: |
|   双层数组之间    |   曲率8186-4012   |   46   |     12.89-14.75     |      0.25       | $\Delta$r:0.237388, $\Delta$t:3.68685 |
|   单层数组之间    |   曲率8186-4012   | 46->21 |     12.89-14.75     |       0.1       | $\Delta$r:0.209357, $\Delta$t:1.12613 |
| 双层数组+半径限制 |   曲率8186-4012   |   64   |     12.89-14.75     |      1.31       | $\Delta$r:0.288036, $\Delta$t:2.92524 |
| 单层数组+半径限制 |   曲率8186-4012   |   33   |     12.89-14.75     |      0.42       | $\Delta$r:0.253338, $\Delta$t:2.73962 |

参数设置：

```
featureOpt.bscOptions.blockResolution = 15.0f;
featureOpt.bscOptions.downsampleResolution = 0.05f;
featureOpt.bscOptions.radius = 0.8f;
featureOpt.bscOptions.voxelNum = 7;
featureOpt.vladOptions.feOpt.visualWordsNum = 100;
featureOpt.vladOptions.feOpt.maxTrainingDataNum = 100000;
optimization::PairwiseRgOpt pwRgopt;
pwRgopt.coRgopt.bscOpt.visualWordNum = featureOpt.vladOptions.feOpt.visualWordsNum;
pwRgopt.coRgopt.bscOpt.tolerantHammingDis = 0.25f;
pwRgopt.coRgopt.bscOpt.tolerantEuclideanDis = 0.2f;
pwRgopt.coRgopt.bscOpt.minFeatNum = 10;
// key points type
bool is_curvature = config["KeyPointType"].as<std::string>() == "iss" ? false : true;
if (!is_curvature)
{ // iss
    featureOpt.bscOptions.kpOption.radiusFeatureCalculation = 0.046f * 6 * 1.5;
    featureOpt.bscOptions.kpOption.radiusNonMax = 0.046f * 4 * 1.5;
    featureOpt.bscOptions.kpOption.ratioMax = 0.975f;
    featureOpt.bscOptions.kpOption.minPtNum = 50;
}
else
{ // curvature
    featureOpt.bscOptions.kpOption.radiusFeatureCalculation = 0.4f;
    featureOpt.bscOptions.kpOption.radiusNonMax = 0.5f;
    featureOpt.bscOptions.kpOption.ratioMax = 0.25f;
    featureOpt.bscOptions.kpOption.minPtNum = 20;
}
```

#### 简单分析

TLS本身没有绝对坐标，而MLS一般是有的，因此两者之间的初始位置偏差本身就比较大。**使用预先设定的半径来搜索同名点是不行的**，在特征空间进行匹配是比较好的。

增大搜索半径会导致性能下降。并且搜索半径会导致计算结果之间存在明显差异，找到一个trade-off的阈值并不容易。

### TLS&TLS点云配准

### MLS&MLS点云配准

+ 同模态数据之间可能初值比较好，但是如果跨源数据可以匹配的比较好，那么同源也可以，前提是**同源数据一定是可以看成刚体**。
+ 相同的代码不同的数据，和手工标记的GT相比，BSC在MLS数据的配准上表现非常好。不同的实现方式之间有一些差别。
+ 平台I7-9700K,3.6GHz。源点云点数：414K，10.2Mb，目标点云点数：227K， 5.6Mb。

#### 可视化

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/all_in_one.png)

#### 定量的指标

|   不同实现方式    | 关键点数量MLS-MLS | 匹配对 | 特征提取耗时s(有IO) | 匹配耗时s(无IO) |           精度           |
| :---------------: | :---------------: | :----: | :-----------------: | :-------------: | :----------------------: |
|   双层数组之间    |    曲率781-537    |   29   |      1.07-0.61      |      0.04       | r:0.0484566, t:0.0444514 |
|   单层数组之间    |    曲率781-537    | 16->11 |      1.07-0.61      |      0.02       |  r:0.342641, t:0.639705  |
| 双层数组+半径限制 |    曲率781-537    |   34   |      1.07-0.61      |      0.05       | r:0.0395647, t:0.0808861 |
| 单层数组+半径限制 |    曲率781-537    |   11   |      1.07-0.61      |      0.03       |  r:0.521896, t:0.411523  |

#### 简单分析

可以发现，匹配对的数量不能太少也不能太多，处于中间的最好。从结果来看20以下的结果明显不如20以上的匹配结果好，而其他方面的开销似乎都差不多。

一个算法的好坏和代码实现上是相关的，虽然核心原理一样，但是不同的实现方式会引入难以控制的误差。

## GROR外点剔除方法实际测试

测试论文：A New Outlier Removal Strategy Based on Reliability of Correspondence Graph for Fast Point Cloud Registration, TPAMI 2022. [official code](https://github.com/WPC-WHU/GROR)

+ 官方测试了WHU-TLS，3D-match等各种各样的数据集，因此直接拿MLS和TLS进行测试。GROR是直接在FPFH+KNN的结果上进一步优化内点数量的。

### MLS&MLS点云配准1

#### 可视化图

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/fpfh_gror_mls&mls_all_in_one.png)

#### 定量的指标

| 不同实现方式 | 关键点数量MLS-MLS | 匹配对 | 特征提取耗时s(有IO) | 匹配耗时s(无IO) |
| :----------: | :---------------: | :----: | :-----------------: | :-------------: |
| FPFH+RANSAC  |  曲率14647-9131   |  2168  |        5.62         |      0.96       |
|   FPFH+KNN   |  曲率14647-9131   |   -    |        5.62         |        -        |
|  FPFH+GROR   |  曲率14647-9131   |   35   |        5.62         |      2.85       |

### MLS&MLS点云配准2

+ 数据同上单线稠密MLS激光点云数据。

#### 可视化图

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/fpfh_gror_mls&mls2_all_in_one.png)

#### 定量的指标

| 不同实现方式 | 关键点数量MLS-MLS | 匹配对 | 特征提取耗时s(有IO) | 匹配耗时s(无IO) |
| :----------: | :---------------: | :----: | :-----------------: | :-------------: |
| FPFH+RANSAC  |  曲率30300-39513  |  9737  |       165.62        |      1.96       |
|   FPFH+KNN   |  曲率30300-39513  |   -    |       165.62        |        -        |
|  FPFH+GROR   |  曲率30300-39513  |  417   |       165.62        |      65.46      |

### TLS&MLS点云配准

默认参数失败了。resolution调整到0.2m之后成功了，但是结果仍然不是足够优秀。连续测试了0.15-0.12，效果都一般。

考虑到FPFH的初始匹配关系太差了，FPFH的计算非常考验参数的选择，选择更强的BSC进行描述和测试。

#### 可视化图

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/bsc_gror_tls&mls_all_in_one.png)

#### 定量的指标

| 不同实现方式 | 关键点数量MLS-MLS | 匹配对 | 特征提取耗时s(有IO) | 匹配耗时s(无IO) |
| :----------: | :---------------: | :----: | :-----------------: | :-------------: |
|     BSC      |  曲率69188-49103  |  1997  |          -          |      6.45       |
|   BSC+GROR   |  曲率69188-49103  |  766   |          -          |      0.41       |

#### 简单分析

+ GROR依赖初始特征描述子提供的匹配结果，GROR本身研究的是如何从包含海量外点的匹配关系中找到对的那部分内点。选择了FPFH作为基础特征描述子，并和FPFH+RANSAC的方法进行了比较。只要有初始的对应关系，后续就能进一步进行剔除，从而找到最佳的匹配内点。
+ FPFH在点云配准中的效果不好，但是引入GROR之后，MLS匹配效果炸裂的好，而且参数都不怎么需要调整。
+ 在困难场景中，e.g. TLS&MLS融合，发现FPFH+GROR也无法满足需求，将FPFH替换为BSC，并放宽BSC匹配阈值（即牺牲部分精度下采样分辨率为0.05m，提升内点基数，方便GROR找到内点），再使用GROR进行剔除。结果如期待的样子，效果非常好。
+ 简单场景FPFH+GROR足够，困难场景需要提升描述子的描述能力，即匹配的内点率，利用GROR强大的外点剔除能力，可以实现比较好的配准结果。反过来说，本身BSC就比较好，使用GROR可以降低BSC对参数的敏感性。
+ **后期有潜力形成一种工程上完全可行的方法**。！！！

+ 参数选择文章中推荐的设置。

```c++
float resolution = 0.1f;（几乎只需要调整这一个参数，因为其他参数都默认写死了。。）
Eigen::Vector3f voxel_size(resolution, resolution, resolution);
Util::PreProcess<PointT>::Ptr processer = std::make_unique<Util::PreProcess<PointT>>(voxel_size);
processer->applyApproximateVoxelFilter(pc1_o, pc1);
processer->applyApproximateVoxelFilter(pc2_o, pc2);
// FPFH registration
Eigen::Matrix4f transformation_matrix_src2tgt = Eigen::Matrix4f::Identity();
featureExtraction::FeatureOption foption;
foption.fpfhOptions.kpOption.radiusFeatureCalculation = 6.f * resolution;
foption.fpfhOptions.kpOption.ratioMax = 0.975;
foption.fpfhOptions.kpOption.radiusNonMax = 4.f * resolution;
foption.fpfhOptions.feature_radius = 3.f * resolution;
foption.fpfhOptions.normal_raidus = 8.f * resolution;
foption.fpfhOptions.RANSAC_Inlier_Threshold = 0.1f;
foption.fpfhOptions.CorrRejDist_Maximum_Distance = 0.1f; 
foption.fpfhOptions.RANSAC_Iterations = 5000;
```

## 基于双平面的点云配准方法实际测试

测试论文：Pairwise coarse registration of point clouds by traversing voxel-based 2-plane bases, IJRS 2022. [unofficial code](https://github.com/amine-chaabouni/pointcloud_registration_two_plane_base)

### MLS&MLS点云配准

#### 可视化图

