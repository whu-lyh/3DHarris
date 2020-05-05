//
//  groundExtracting.h
//  para
//
//  Created by 虞敏 on 2020/4/21.
//  Copyright © 2020 Moyna. All rights reserved.
//
#pragma once
#ifndef groundExtracting_h
#define groundExtracting_h

#include "dataStructure.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>


namespace pmProcessUrban
{
    class CGroundExtraction
    {
    public:
        
        CGroundExtraction();
        ~CGroundExtraction();
        /*地面点提取,该函数用来识别地面点和非地面点;*/
        //ym在这个函数中加一个参数vector<int>groundpts_indexs
        void ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
                                Bounds_ym bounds, CenterPoint_ym center_pt,vector<int> &groundpts_indexs);
        
        void SetMaxHeightDifference(float max_height_difference) { max_height_difference_ = max_height_difference; }
        void SetMinPointNumInGrid(int min_pt_num_in_grid) { min_pt_num_in_grid_ = min_pt_num_in_grid; }
        void SetGridResolution(float grid_resolution) { grid_resolution_ = grid_resolution; }
        
    protected:
        
    private:
        
        /*计算格网中的最低点高程;*/
        void CalculateGridAndMinZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                  double max_x, double max_y, double min_x, double min_y,
                                  int row, int list, int num_voxel, Voxel* grid);
        
        /*根据点跟Grid中最低点的高程差，判断地面点和非地面点;*/
        //ym在这个函数中加一个参数vector<int>groundpts_indexs
        void JudgeGroundAndNonGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
                                           Voxel* grid, int num_voxel,std::vector<int> &groundpts_indexs);
        
        
        /*对地面点进一步判断，如果该地面点高于周围地面点很多，则判断为非地面点;*/
        void FilterNotGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud);
        
        float grid_resolution_;//格网的分辨率;
        int   min_pt_num_in_grid_;//格网中最小的点个数，小于该点数据认为该格网为噪声点;
        float max_height_difference_;//个网内的点与个网内最低点的高程差,大于该值被认为是非地面点;
        
        
    };
}

#endif /* groundExtracting_h */
