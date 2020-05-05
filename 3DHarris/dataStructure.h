//
//  dataStructure.h
//  para
//
//  Created by 虞敏 on 2020/4/20.
//  Copyright © 2020 Moyna. All rights reserved.
//
#pragma once
#ifndef dataStructure_h
#define dataStructure_h


#include <vector>
#include <list>
#include <string>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;

namespace pmProcessUrban
{
    struct CenterPoint_ym
    {
        double x;
        double y;
        double z;
        CenterPoint_ym(double x = 0, double y = 0, double z = 0) :
        x(x), y(y), z(z)
        {
            z = 0.0;
            x = y = 0.0;
        }
        
    };
    
    struct Bounds_ym
    {
        double min_x;
        double min_y;
        double min_z;
        double max_x;
        double max_y;
        double max_z;
        Bounds_ym()
        {
            min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
        }
    };
    
    struct Grid
    {
        bool is_empty;
        Grid()
        {
            is_empty = true;
        }
    };
    
    struct Voxel
    {
        vector<int>point_id;
        float min_z;
        float max_z;
        float dertaz;
        float min_z_x;//格网最低点的X坐标;
        float min_z_y;//格网最低点的y坐标;
        float NeighborMin_z;
        int PointsNumber;
        float mean_z;
        
        Voxel()
        {
            min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
            PointsNumber = 1;
            dertaz = 0.0;
        }
    };
    
    
      
        
}

#endif /* dataStructure_h */
