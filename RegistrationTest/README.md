## 基于平面的点云配准方法实际测试

Point cloud registration and localization based on voxel plane features

### 官方的demo数据和默认参数

```bash
transformation_matrix:
 -0.931591  -0.363332  0.0112994    1.01878
  0.363149  -0.931602 -0.0155058    8.16221
 0.0161603 -0.0103416   0.999816  0.0262827
         0          0          0          1
```

与CloudCompare的Trimm-ICP的结果：

```bash
Final RMS*: 0.600062 (computed on 33041 points)(* RMS is potentially weighted, depending on the selected options)
Transformation matrix：
0.933	-0.360 	0.008	-4.308
0.360	0.933	0.014	-1.921	
-0.013	-0.010 	1.000	-0.042
0.000	0.000	0.000	1.000
Scale: fixed (1.0)
Theoretical overlap: 90%
This report has been output to Console (F8)
```

明显可以发现T-icp，90%重叠下的结果中有明显错乱，而这个方法表现就非常好了（除了点云中存在的非刚性畸变）。**如果在VPFB-R之后再接一个T-ICP的方法（90%重叠度），可以发现效果会好非常多，那么整体就是coarse-to-fine的方案，也能保证结果不错**。

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/demo_total.png)

### 真实MLS点云数据测试

```
transformation_matrix:
    0.999997   0.00242054 -0.000144257     0.190775
  -0.0024208     0.999995  -0.00181927    -0.420039
 0.000139853   0.00181962     0.999998   0.00553238
           0            0            0            1
```

![image-20230711123258963](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/image-20230711123258963.png)

+ 红色为初始配准之后的结果，可以发现由于场景中平面对数量不足，导致匹配结果容易出现不稳定的情况，对场景比较依赖。

+ 简单调整参数之后，选择平面数量越多可以保证结果比较好，其他参数没什么变化。

+ 尝试去掉地面，发现地面效果去除的还挺好的，但是配准效果也不怎么样。

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/Section0_non_ground.png)

![](https://cdn.jsdelivr.net/gh/whu-lyh/images//img/Section61_non_ground.png)

+ 重新调整了一个k1参数，发现效果还可以的。而这个和平面有无关系不大了。

+ 在跨源点云匹配中发现参数又不行了。



