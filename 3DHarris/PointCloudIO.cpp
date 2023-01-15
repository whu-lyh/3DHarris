#pragma once
#ifdef TEMPLATE_POINTCLOUD_IO

#include "basicLibs.h"
#include "Utility.h"
#include "PointCloudIO.h"
#include "PointType.h"

#include <pcl/io/pcd_io.h>

using namespace Utility;

namespace PointIO
{
	// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
	// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
	template<typename T>
	double computeCloudResolution(const typename pcl::PointCloud<T>::ConstPtr& cloud)
	{
		double resolution = 0.0;
		int numberOfPoints = 0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> squaredDistances(2);
		pcl::search::KdTree<T> tree;
		tree.setInputCloud(cloud);

		for ( size_t i = 0; i < cloud->size(); ++i )
		{
			if ( !pcl_isfinite((*cloud)[i].x) )
				continue;

			// Considering the second neighbor since the first is the point itself.
			// return number of neighbors found
			nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
			if ( nres == 2 )
			{
				resolution += sqrt(squaredDistances[1]);
				++numberOfPoints;
			}
		}
		if ( numberOfPoints != 0 )
			resolution /= numberOfPoints;

		return resolution;
	}
	
	template <typename T>
	int generateStdColorRamp(typename std::vector<std::vector<T>> &colorRamp)
	{
		colorRamp.emplace_back(std::vector<T>{ 0.0, 0.0, 0.0 });
		colorRamp.emplace_back(std::vector<T>{0.0, 255.0, 255.0});
		colorRamp.emplace_back(std::vector<T>{0.0, 255.0, 0.0});
		colorRamp.emplace_back(std::vector<T>{255.0, 255.0, 0.0});
		colorRamp.emplace_back(std::vector<T>{255.0, 0.0, 0.0});
		return 1;
	}

	template <typename T>
	bool setColorByDistance(typename std::vector<std::vector<T>> &colorRamp, const double &maxdist, const double &mindist, const double &distance, T &r, T &g, T &b)
	{ //set color by distance and the larger distance , the deeper color will be set
		double rampLength = maxdist - mindist;
		double sectionLength = double(rampLength) / colorRamp.size();

		double relevant_val = distance - mindist;
		int index(relevant_val / sectionLength);
		int maxIndex = colorRamp.size() - 1;
		if (index >= maxIndex)
		{//the higher height
			index = colorRamp.size() - 1;
			r = colorRamp[index][0];
			g = colorRamp[index][1];
			b = colorRamp[index][2];
		}
		else if (index < 0)	//考虑噪点小于统计出的最低距离的情况
		{//the lower height
			index = 0;
			r = colorRamp[index][0];
			g = colorRamp[index][1];
			b = colorRamp[index][2];
		}
		else
		{
			double remainder = relevant_val - sectionLength * index;
			double ratio = (double)remainder / sectionLength;

			r = (colorRamp[index + 1][0] - colorRamp[index][0]) * ratio + colorRamp[index][0];
			g = (colorRamp[index + 1][1] - colorRamp[index][1]) * ratio + colorRamp[index][1];
			b = (colorRamp[index + 1][2] - colorRamp[index][2]) * ratio + colorRamp[index][2];
		}

		return true;
	}

	//set the distance as a property to each point
	template<typename T, size_t D>
	double computeCloudResolution(const typename pcl::PointCloud<T>::ConstPtr& cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color)
	{
		double resolution = 0.0;
		int numberOfPoints = 0;
		int nres;
		std::vector<int> indices(D);
		std::vector<float> squaredDistances(D);
		pcl::search::KdTree<T> tree;
		tree.setInputCloud(cloud);

		//generate the std color ramp set
		std::vector<std::vector<float>> colorRamp;
		PointIO::generateStdColorRamp<float>(colorRamp);

		pcl::PointXYZRGB pt_color;
		std::vector<float> distance_vec;
		double max_dist = std::numeric_limits<float>::lowest(), min_dist = std::numeric_limits<float>::max();
		for (int i = 0; i < cloud->size(); ++i)
		{
			if (!pcl_isfinite((*cloud)[i].x))
				continue;

			// Considering the second neighbor since the first is the point itself.
			// return number of neighbors found
			nres = tree.nearestKSearch(i, D, indices, squaredDistances);
			if (nres == D)
			{
				float single_distance = 0.0;
				for (auto sd : squaredDistances)
				{
					single_distance += sd;
				}
				
				resolution += sqrt(squaredDistances[1]);

				if (single_distance > max_dist) max_dist = std::log(single_distance);
				if (single_distance < min_dist) min_dist = std::log(single_distance);
				distance_vec.emplace_back(std::log(single_distance));

				++numberOfPoints;
			}
		}
		std::cout << "max dist: " << max_dist << ",\t" << "min dist: " << min_dist << std::endl;
//#pragma omp parallel for
		for (int i = 0; i < cloud->size(); ++i)
		{
			if (!pcl_isfinite((*cloud)[i].x))
				continue;

			pcl::copyPoint((*cloud)[i], pt_color);

			float r = 0, g = 0, b = 0;
			PointIO::setColorByDistance<float>(colorRamp, max_dist - min_dist, 0, distance_vec[i] - min_dist, r, g, b);
			pt_color.r = r;
			pt_color.g = g;
			pt_color.b = b;

			cloud_color->push_back(pt_color);
		}

		if (numberOfPoints != 0)
			resolution /= numberOfPoints;

		return resolution;
	}

	template<typename T>
	Utility::Bound getBoundBox<T>(const typename pcl::PointCloud<T>::Ptr& cloud)
	{
		Utility::Bound bound;
		bound.min_x = std::numeric_limits<double>::max();
		bound.min_y = std::numeric_limits<double>::max();
		bound.min_z = std::numeric_limits<double>::max();
		bound.max_x = std::numeric_limits<double>::lowest();
		bound.max_y = std::numeric_limits<double>::lowest();
		bound.max_z = std::numeric_limits<double>::lowest();

		if (!cloud)
		{
			std::cerr << "cloud is null!";
			return bound;
		}

		for (auto pt : cloud->points)
		{
			if (pt.x < bound.min_x)
				bound.min_x = pt.x;
			if (pt.x > bound.max_x)
				bound.max_x = pt.x;
			if (pt.y < bound.min_y)
				bound.min_y = pt.y;
			if (pt.y > bound.max_y)
				bound.max_y = pt.y;
			if (pt.z < bound.min_z)
				bound.min_z = pt.z;
			if (pt.z > bound.max_z)
				bound.max_z = pt.z;
		}
		return bound;
	}

	template <typename T>
	void parseLAS(LASreader &lasreader, const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
	{
	}

	template <>
	inline void parseLAS<PointXYZRGBINTF>(LASreader &lasreader, const pcl::PointCloud<PointXYZRGBINTF>::Ptr& cloud, const Offset& offset)
	{
		if (lasreader.header.point_data_format != 3 || lasreader.header.point_data_format != 7)
		{
			LOG(ERROR) << "las file data formate is wrong, please check, current las format is 1." << (unsigned)lasreader.header.point_data_format;
			return;
		}
		else
		{
			while (lasreader.read_point())
			{
				PointXYZRGBINTF pt;
				pt.x = lasreader.point.get_x() - offset.x;
				pt.y = lasreader.point.get_y() - offset.y;
				pt.z = lasreader.point.get_z() - offset.z;
				pt.r = lasreader.point.get_R() >> 8;
				pt.g = lasreader.point.get_G() >> 8;
				pt.b = lasreader.point.get_B() >> 8;
				pt.intensity = lasreader.point.get_intensity();
				pt.num_of_returns = lasreader.point.get_number_of_returns();
				pt.return_number = lasreader.point.get_return_number();
				pt.gps_time = lasreader.point.get_gps_time();
				pt.flighting_line_edge = lasreader.point.get_edge_of_flight_line();
				pt.classification = lasreader.point.get_classification();
				cloud->push_back(pt);
			}
		}
	}

	template <>
	inline void parseLAS<PointXYZINTF>(LASreader &lasreader, const pcl::PointCloud<PointXYZINTF>::Ptr& cloud, const Offset& offset)
	{
		if (lasreader.header.point_data_format == 0 && lasreader.header.point_data_format == 2)
		{
			LOG(ERROR) << "las file data formate is wrong, please check, current las format is 1. " << (unsigned)lasreader.header.point_data_format;
			return;
		}
		else
		{
			while (lasreader.read_point())
			{
				PointXYZINTF pt;
				pt.x = lasreader.point.get_x() - offset.x;
				pt.y = lasreader.point.get_y() - offset.y;
				pt.z = lasreader.point.get_z() - offset.z;
				pt.intensity = lasreader.point.get_intensity(); // typedef unsigned short     U16;
				pt.num_of_returns = lasreader.point.get_number_of_returns(); // typedef unsigned char      U8;
				pt.return_number = lasreader.point.get_return_number(); //unsigned char = U8 = uint8_t
				pt.gps_time = lasreader.point.get_gps_time();
				pt.flighting_line_edge = lasreader.point.get_edge_of_flight_line();
				pt.classification = lasreader.point.get_classification();
				cloud->push_back(pt);
			}
		}
	}

	template <>
	inline void parseLAS<pcl::PointXYZ>(LASreader &lasreader, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Offset& offset)
	{
		while (lasreader.read_point())
		{
			pcl::PointXYZ pt;
			pt.x = lasreader.point.get_x() - offset.x;
			pt.y = lasreader.point.get_y() - offset.y;
			pt.z = lasreader.point.get_z() - offset.z;
			cloud->push_back(pt);
		}
	}

	template <>
	inline void parseLAS<PointXYZT>(LASreader &lasreader, const pcl::PointCloud<PointXYZT>::Ptr& cloud, const Offset& offset)
	{
		while (lasreader.read_point())
		{
			PointXYZT pt;
			pt.x = lasreader.point.get_x() - offset.x;
			pt.y = lasreader.point.get_y() - offset.y;
			pt.z = lasreader.point.get_z() - offset.z;
			pt.gps_time = lasreader.point.get_gps_time();
			cloud->push_back(pt);
		}
	}

	template <>
	inline void parseLAS<pcl::PointXYZRGB>(LASreader &lasreader, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Offset& offset)
	{
		if (lasreader.header.point_data_format != 2 && lasreader.header.point_data_format != 3
			&& lasreader.header.point_data_format != 7 && lasreader.header.point_data_format != 8)
		{
			LOG(ERROR) << "las file data formate is wrong, please check, current las format is 1. " << (unsigned)lasreader.header.point_data_format;
			return;
		}
		else
		{
			while (lasreader.read_point())
			{
				pcl::PointXYZRGB pt;
				pt.x = lasreader.point.get_x() - offset.x;
				pt.y = lasreader.point.get_y() - offset.y;
				pt.z = lasreader.point.get_z() - offset.z;
				pt.r = lasreader.point.get_R() >> 8; //from u16 to uint8_t
				pt.g = lasreader.point.get_G() >> 8;
				pt.g = lasreader.point.get_B() >> 8;
				cloud->push_back(pt);
			}
		}
	}

	template <typename T>
	bool loadSingleLAS(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& las_offset)
	{
		if (!file_exist(filename))
		{
			LOG(ERROR) << filename << " doesn't exist!";
			return false;
		}
		if (is_directory(filename))
		{
			LOG(ERROR) << filename << " is a directory!";
			return false;
		}
		if (cloud == nullptr)
		{
			LOG(ERROR) << "pointer cloud is nullptr!";
			return false;
		}
		try
		{
			// check extension
			std::string ext = boost::filesystem::extension(filename);
			if (ext.compare(".las"))
				LOG(ERROR) << "It's a inappropriate file format.";
			// open las file
			LASreadOpener lasreadopener;
			lasreadopener.set_file_name(filename.c_str());
			if (!lasreadopener.active())
			{
				LOG(ERROR) << "No input specified " << filename;
			}
			LASreader* lasreader = lasreadopener.open();
			if (lasreader == 0)
			{
				LOG(ERROR) << "Could not open lasreader";
			}
			uint8_t major_version = lasreader->header.version_major;
			uint8_t minor_version = lasreader->header.version_minor;
			if (major_version > 1 || minor_version > 4)
			{
				LOG(ERROR) << "Currently this app doesn't support version newer than 1.4";
				return false;
			}
			// Get info from reader.header
			uint32_t pts_count = lasreader->header.number_of_point_records;
			uint32_t offset = lasreader->header.offset_to_point_data;
			uint16_t pt_length = lasreader->header.point_data_record_length; // uint16_t=U16
			U8 las_format = lasreader->header.point_data_format;
			double offset_x = lasreader->header.x_offset;
			double offset_y = lasreader->header.y_offset;
			double offset_z = lasreader->header.z_offset;
			las_offset = Offset(offset_x, offset_y, offset_z);
			parseLAS<T>(*lasreader, cloud, las_offset);
			// close lasreader
			lasreader->close();
			delete lasreader;
		}
		catch (std::exception* e)
		{
			LOG(ERROR) << "Error occurred when parsing las file: " << e->what();
		}
		return true;
	}

	template <typename T>
	bool loadLASFromFolder(const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud)
	{
		//TODO: load las files from folder
		return true;
	}

	template <typename T>
	bool saveLAS2(const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
	{
		return true;
	}

	template <>
	inline bool saveLAS2<PointXYZRGBINTF>(const std::string& filepath, const typename pcl::PointCloud<PointXYZRGBINTF>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			LOG(ERROR) << "Pointer cloud is nullptr!";
			return false;
		}
		if (cloud->empty())
		{
			LOG(ERROR) << "Point cloud is empty!";
			return false;
		}
		// LASlib write las file
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(filepath.c_str());
		if (!laswriteopener.active())
		{
			LOG(ERROR) << "No out put file specified! " << filepath;
			return false;
		}
		LASheader *lasheader = new LASheader();
		if (lasheader == 0)
		{
			LOG(ERROR) << "Could not open lasreader";
			return false;
		}
		Utility::Bound boundbox = getBoundBox<PointXYZRGBINTF>(cloud);
		// Normal header setting
		strncpy(lasheader->system_identifier, "Group.Yang", 11);
		lasheader->system_identifier[10] = '\0';
		strncpy(lasheader->generating_software, "2.0", 4);
		lasheader->generating_software[3] = '\0';
		lasheader->version_major = 1;
		lasheader->version_minor = 2;
		lasheader->set_bounding_box(boundbox.min_x + offset.x, boundbox.min_y + offset.y, boundbox.min_z + offset.z,
			boundbox.max_x + offset.x, boundbox.max_x + offset.y, boundbox.max_x + offset.z,
			false, false);
		lasheader->header_size = 227;
		lasheader->offset_to_point_data = 227;
		// XYZRGBINTF
		lasheader->point_data_format = 3;
		lasheader->point_data_record_length = 34;
		// Geometry info
		lasheader->number_of_point_records = (U32)cloud->size();
		lasheader->x_scale_factor = 0.0001;
		lasheader->y_scale_factor = 0.0001;
		lasheader->z_scale_factor = 0.0001;
		lasheader->x_offset = offset.x;
		lasheader->y_offset = offset.y;
		lasheader->z_offset = offset.z;
		LASwriter* laswriter = laswriteopener.open(lasheader);
		if (laswriter == 0)
		{
			LOG(ERROR) << "Could not open laswriter";
			return false;
		}
		LASpoint *point = new LASpoint();
		point->init(lasheader, lasheader->point_data_format, lasheader->point_data_record_length, 0);
		// write point clouds to las file with rgb
		for (int i = 0; i < cloud->size(); ++i)
		{
			double x = static_cast<double>(cloud->points[i].x) + offset.x;
			double y = static_cast<double>(cloud->points[i].y) + offset.y;
			double z = static_cast<double>(cloud->points[i].z) + offset.z;
			// here the coordinate just have offset with out scale, so should use set_x instead of set_X
			point->set_x(x);
			point->set_y(y);
			point->set_z(z);
			point->rgb[0] = U16_QUANTIZE(cloud->points[i].r);
			point->rgb[1] = U16_QUANTIZE(cloud->points[i].g);
			point->rgb[2] = U16_QUANTIZE(cloud->points[i].b);
			point->set_intensity(U16_QUANTIZE(cloud->points[i].intensity));
			point->set_return_number(U8_QUANTIZE(cloud->points[i].return_number));
			point->set_number_of_returns(U8_QUANTIZE(cloud->points[i].num_of_returns));
			point->set_gps_time(cloud->points[i].gps_time);
			point->set_edge_of_flight_line(U8_QUANTIZE(cloud->points[i].flighting_line_edge));
			point->set_classification(U8_QUANTIZE(cloud->points[i].classification));
			// write the modified point
			laswriter->write_point(point);
			laswriter->update_inventory(point);
		}
		laswriter->update_header(lasheader, TRUE);
		I64 total_bytes = laswriter->close();
		if (laswriter == 0)
		{
			LOG(ERROR) << "ERROR: could not open laswriter";
		}
		delete laswriter;
		return true;
	}

	template <>
	inline bool saveLAS2<PointXYZINTF>(const std::string& filepath, const typename pcl::PointCloud<PointXYZINTF>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			LOG(ERROR) << "Pointer cloud is nullptr!";
			return false;
		}
		if (cloud->empty())
		{
			LOG(ERROR) << "Point cloud is empty!";
			return false;
		}
		Utility::Bound boundbox = getBoundBox<PointXYZINTF>(cloud);
		// LASlib write las file
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(filepath.c_str());
		if (!laswriteopener.active())
		{
			LOG(ERROR) << "No out put file specified! " << filepath;
			return false;
		}
		LASheader *lasheader = new LASheader();
		if (lasheader == 0)
		{
			LOG(ERROR) << "Could not open lasreader";
			return false;
		}
		// Normal header setting
		strncpy(lasheader->system_identifier, "Group.Yang", 11);
		lasheader->system_identifier[10] = '\0';
		strncpy(lasheader->generating_software, "2.0", 4);
		lasheader->generating_software[3] = '\0';
		lasheader->version_major = 1;
		lasheader->version_minor = 2;
		lasheader->set_bounding_box(boundbox.min_x + offset.x, boundbox.min_y + offset.y, boundbox.min_z + offset.z,
			boundbox.max_x + offset.x, boundbox.max_x + offset.y, boundbox.max_x + offset.z,
			false, false);
		lasheader->header_size = 227;
		lasheader->offset_to_point_data = 227;
		// XYZINTF
		lasheader->point_data_format = 1;
		lasheader->point_data_record_length = 28;
		// Geometry info
		lasheader->number_of_point_records = (U32)cloud->size();
		lasheader->x_scale_factor = 0.0001;
		lasheader->y_scale_factor = 0.0001;
		lasheader->z_scale_factor = 0.0001;
		lasheader->x_offset = offset.x;
		lasheader->y_offset = offset.y;
		lasheader->z_offset = offset.z;
		LASwriter* laswriter = laswriteopener.open(lasheader);
		if (laswriter == 0)
		{
			LOG(ERROR) << "Could not open laswriter\n";
			return false;
		}
		LASpoint *point = new LASpoint();
		point->init(lasheader, lasheader->point_data_format, lasheader->point_data_record_length, 0);
		// write point clouds to las file with rgb
		for (int i = 0; i < cloud->size(); ++i)
		{
			double x = static_cast<double>(cloud->points[i].x) + offset.x;
			double y = static_cast<double>(cloud->points[i].y) + offset.y;
			double z = static_cast<double>(cloud->points[i].z) + offset.z;
			// here the coordinate just have offset with out scale, so should use set_x instead of set_X
			point->set_x(x);
			point->set_y(y);
			point->set_z(z);
			point->set_intensity(U16_QUANTIZE(cloud->points[i].intensity));
			point->set_number_of_returns(U8_QUANTIZE(cloud->points[i].num_of_returns));
			point->set_return_number(U8_QUANTIZE(cloud->points[i].return_number));
			point->set_gps_time(cloud->points[i].gps_time);
			point->set_edge_of_flight_line(U8_QUANTIZE(cloud->points[i].flighting_line_edge));
			point->set_classification(U8_QUANTIZE(cloud->points[i].classification));
			// write the modified point
			laswriter->write_point(point);
			laswriter->update_inventory(point);
		}
		laswriter->update_header(lasheader, TRUE);
		I64 total_bytes = laswriter->close();
		if (laswriter == 0)
		{
			LOG(ERROR) << "ERROR: could not open laswriter\n";
		}
		delete laswriter;
		return true;
	}

	template <>
	inline bool saveLAS2<pcl::PointXYZRGB>(const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			LOG(ERROR) << "Pointer cloud is nullptr!";
			return false;
		}
		Utility::Bound boundbox = PointIO::getBoundBox<pcl::PointXYZRGB>(cloud);
		if (cloud->empty())
		{
			LOG(ERROR) << "Point cloud is empty!";
			return false;
		}
		// LASlib write las file
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(filepath.c_str());
		if (!laswriteopener.active())
		{
			LOG(ERROR) << "No out put file specified! " << filepath;
			return false;
		}
		LASheader *lasheader = new LASheader();
		if (lasheader == 0)
		{
			LOG(ERROR) << "Could not open lasreader";
			return false;
		}
		// Normal header setting
		strncpy(lasheader->system_identifier, "Group.Yang", 11);
		lasheader->system_identifier[10] = '\0';
		strncpy(lasheader->generating_software, "2.0", 4);
		lasheader->generating_software[3] = '\0';
		lasheader->version_major = 1;
		lasheader->version_minor = 2;
		lasheader->set_bounding_box(boundbox.min_x + offset.x, boundbox.min_y + offset.y, boundbox.min_z + offset.z,
			boundbox.max_x + offset.x, boundbox.max_x + offset.y, boundbox.max_x + offset.z,
			false, false);
		lasheader->header_size = 227;
		lasheader->offset_to_point_data = 227;
		// XYZRGB
		lasheader->point_data_format = 2;
		lasheader->point_data_record_length = 26;
		// Geometry info
		lasheader->number_of_point_records = (U32)cloud->size();
		lasheader->x_scale_factor = 0.0001;
		lasheader->y_scale_factor = 0.0001;
		lasheader->z_scale_factor = 0.0001;
		lasheader->x_offset = offset.x;
		lasheader->y_offset = offset.y;
		lasheader->z_offset = offset.z;
		LASwriter* laswriter = laswriteopener.open(lasheader);
		if (laswriter == 0)
		{
			LOG(ERROR) << "Could not open laswriter";
			return false;
		}
		LASpoint *point = new LASpoint();
		point->init(lasheader, lasheader->point_data_format, lasheader->point_data_record_length, 0);
		// write point clouds to las file with rgb
		for (int i = 0; i < cloud->size(); ++i)
		{
			double x = static_cast<double>(cloud->points[i].x) + offset.x;
			double y = static_cast<double>(cloud->points[i].y) + offset.y;
			double z = static_cast<double>(cloud->points[i].z) + offset.z;
			// here the coordinate just have offset with out scale, so should use set_x instead of set_X
			point->set_x(x);
			point->set_y(y);
			point->set_z(z);
			point->set_intensity(10);
			point->rgb[0] = U16_QUANTIZE(cloud->points[i].r);
			point->rgb[1] = U16_QUANTIZE(cloud->points[i].g);
			point->rgb[2] = U16_QUANTIZE(cloud->points[i].b);
			// write the modified point
			laswriter->write_point(point);
			laswriter->update_inventory(point);
		}
		laswriter->update_header(lasheader, TRUE);
		I64 total_bytes = laswriter->close();
		if (laswriter == 0)
		{
			LOG(ERROR) << "ERROR: could not open laswriter";
		}
		delete laswriter;
		return true;
	}

	template <>
	inline bool saveLAS2<pcl::PointXYZ>(const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			LOG(ERROR) << "Pointer cloud is nullptr!";
			return false;
		}
		Utility::Bound boundbox = PointIO::getBoundBox<pcl::PointXYZ>(cloud);
		if (cloud->empty())
		{
			LOG(ERROR) << "Point cloud is empty!";
			return false;
		}
		// LASlib write las file
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(filepath.c_str());
		if (!laswriteopener.active())
		{
			LOG(ERROR) << "No out put file specified! " << filepath;
			return false;
		}
		LASheader *lasheader = new LASheader();
		if (lasheader == 0)
		{
			LOG(ERROR) << "Could not open lasreader";
			return false;
		}
		// Normal header setting
		strncpy(lasheader->system_identifier, "Group.Yang", 11);
		lasheader->system_identifier[10] = '\0';
		strncpy(lasheader->generating_software, "2.0", 4);
		lasheader->generating_software[3] = '\0';
		lasheader->version_major = 1;
		lasheader->version_minor = 2;
		lasheader->set_bounding_box(boundbox.min_x + offset.x, boundbox.min_y + offset.y, boundbox.min_z + offset.z,
			boundbox.max_x + offset.x, boundbox.max_x + offset.y, boundbox.max_x + offset.z,
			false, false);
		lasheader->header_size = 227;
		lasheader->offset_to_point_data = 227;
		// XYZI
		lasheader->point_data_format = 0;
		lasheader->point_data_record_length = 20;
		// Geometry info
		lasheader->number_of_point_records = (U32)cloud->size();
		lasheader->x_scale_factor = 0.0001;
		lasheader->y_scale_factor = 0.0001;
		lasheader->z_scale_factor = 0.0001;
		lasheader->x_offset = offset.x;
		lasheader->y_offset = offset.y;
		lasheader->z_offset = offset.z;
		LASwriter* laswriter = laswriteopener.open(lasheader);
		if (laswriter == 0)
		{
			LOG(ERROR) << "Could not open laswriter";
			return false;
		}
		LASpoint *point = new LASpoint();
		point->init(lasheader, lasheader->point_data_format, lasheader->point_data_record_length, 0);
		// write point clouds to las file with rgb
		for (int i = 0; i < cloud->size(); ++i)
		{
			double x = static_cast<double>(cloud->points[i].x) + offset.x;
			double y = static_cast<double>(cloud->points[i].y) + offset.y;
			double z = static_cast<double>(cloud->points[i].z) + offset.z;
			// here the coordinate just have offset with out scale, so should use set_x instead of set_X
			point->set_x(x);
			point->set_y(y);
			point->set_z(z);
			// a random intensity value
			point->set_intensity(10);
			// write the modified point
			laswriter->write_point(point);
			laswriter->update_inventory(point);
		}
		laswriter->update_header(lasheader, TRUE);
		I64 total_bytes = laswriter->close();
		if (laswriter == 0)
		{
			LOG(ERROR) << "ERROR: could not open laswriter";
		}
		delete laswriter;
		return true;
	}

	template <typename T>
	bool saveLAS(const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
	{
		return saveLAS2<T>(filepath, cloud, offset);
	}

	template <typename T>
	bool loadPCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud)
	{
		if (cloud == nullptr)
		{
			LOG(ERROR) << "pointer 'cloud' is nullptr!";
			return false;
		}
		if (pcl::io::loadPCDFile(filename, *cloud) != -1)
		{
			return true;
		}
		return false;
	}

	template <typename T>
	bool savePCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud)
	{
		if (cloud == nullptr)
		{
			LOG(ERROR) << "pointer 'cloud' is nullptr!";
			return false;
		}
		if (cloud->empty())
		{
			LOG(ERROR) << "point cloud is empty!";
			return false;
		}
		pcl::io::savePCDFileBinary(filename, *cloud);
		return true;
	}

	//parse spt file < 400MB
	template <typename T>
	void parseSPT(const std::string &filename, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& offset)
	{
	}

	template <>
	inline void parseSPT <pcl::PointXYZ>(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Offset& offset)
	{
		std::ifstream ifs;
		ifs.open(filename, std::ios::in | std::ios::binary);
		// offset
		double* offset_data = new double[3];
		ifs.seekg(0, std::ios::beg);
		ifs.read(reinterpret_cast<char*>(offset_data), 3 * sizeof(double));
		// point size = calculate the size of basic type and divided
		// move the pointer to the end of the binary file
		ifs.seekg(0, std::ios::end);
		int data_length = ifs.tellg();
		int num_pts = (data_length - 3 * sizeof(double)) / (sizeof(float) * 4 + sizeof(double) + sizeof(uint8_t) * 4);
		float* data = new float[num_pts * 3];
		// here jump the pointer to the offset 's end() and read the xyzintf information
		ifs.seekg(3 * sizeof(double), std::ios::beg);
		for (int singlepoint = 0; singlepoint < num_pts; ++singlepoint)
		{
			ifs.read(reinterpret_cast<char*>(&data[3 * singlepoint]), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&data[3 * singlepoint + 1]), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&data[3 * singlepoint + 2]), sizeof(float));
			// move the pointer to skip the intensity, gpstime, number of return, return_number, classification and edge of flight
			ifs.seekg(sizeof(float) + sizeof(double) + sizeof(uint8_t) * 4, std::ios::cur);
		}

		ifs.close();
		offset = Utility::Offset(offset_data[0], offset_data[1], offset_data[2]);
		for (int i = 0; i < num_pts; ++i)
		{
			pcl::PointXYZ pt;
			pt.x = data[3 * i];
			pt.y = data[3 * i + 1];
			pt.z = data[3 * i + 2];
			cloud->push_back(pt);
		}
		delete[] data;
		delete[] offset_data;
	}

	template <>
	inline void parseSPT <PointXYZINTF>(const std::string &filename, const pcl::PointCloud<PointXYZINTF>::Ptr& cloud, Offset& offset)
	{
		std::ifstream ifs;
		ifs.open(filename, std::ios::in | std::ios::binary);
		// offset
		double* offset_data = new double[3];
		ifs.seekg(0, std::ios::beg);
		ifs.read(reinterpret_cast<char*>(offset_data), 3 * sizeof(double));
		// point size = calculate the size of basic type and divided
		// move the pointer to the end of the binary file
		ifs.seekg(0, std::ios::end);
		int data_length = ifs.tellg();
		int num_pts = (data_length - 3 * sizeof(double)) / (sizeof(float) * 4 + sizeof(double) + sizeof(uint8_t) * 4);
		float* data = new float[num_pts * 3];
		float* intendata = new float[num_pts];
		uint8_t* propdata = new uint8_t[num_pts * 4];
		double* timedata = new double[num_pts];
		// here jump the pointer to the offset 's end() and read the xyzintf information
		ifs.seekg(3 * sizeof(double), std::ios::beg);
		for (int singlepoint = 0; singlepoint < num_pts; ++singlepoint)
		{
			ifs.read(reinterpret_cast<char*>(&data[3 * singlepoint]), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&data[3 * singlepoint + 1]), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&data[3 * singlepoint + 2]), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&intendata[singlepoint]), sizeof(float));
			ifs.read(reinterpret_cast<char*>(&propdata[4 * singlepoint]), sizeof(uint8_t));//uint8_t=unsigned char
			ifs.read(reinterpret_cast<char*>(&propdata[4 * singlepoint + 1]), sizeof(uint8_t));
			ifs.read(reinterpret_cast<char*>(&propdata[4 * singlepoint + 2]), sizeof(uint8_t));
			ifs.read(reinterpret_cast<char*>(&propdata[4 * singlepoint + 3]), sizeof(uint8_t));
			ifs.read(reinterpret_cast<char*>(&timedata[singlepoint]), sizeof(double));
		}
		ifs.close();
		offset = Utility::Offset(offset_data[0], offset_data[1], offset_data[2]);
		for (int i = 0; i < num_pts; ++i)
		{
			PointXYZINTF pt;
			pt.x = data[3 * i];
			pt.y = data[3 * i + 1];
			pt.z = data[3 * i + 2];
			pt.intensity = intendata[i];
			pt.num_of_returns = propdata[3 * i];
			pt.return_number = propdata[3 * i + 1];
			pt.flighting_line_edge = propdata[3 * i + 2];
			pt.classification = propdata[3 * i + 3];
			pt.gps_time = timedata[i];
			cloud->push_back(pt);
		}
		delete[] data;
		delete[] offset_data;
		delete[] intendata;
		delete[] propdata;
		delete[] timedata;
	}

	template <>
	inline void parseSPT <PointXYZRGBINTF>(const std::string &filename, const pcl::PointCloud<PointXYZRGBINTF>::Ptr& cloud, Offset& offset)
	{
		//TODO
	}

	// mmap function
	template<typename T>
	void parseSPTmmf(const char *pFile, const uint32_t pts_num, const typename pcl::PointCloud<T>::Ptr& cloud)
	{
	}

	template <>
	inline void parseSPTmmf <pcl::PointXYZ>(const char *pFile, const uint32_t pts_num, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
	{
		uint32_t pt_length = sizeof(float) * 4 + sizeof(double) + sizeof(uint8_t) * 4;
		float* data = new float[3];
		for (int i = 0; i < pts_num; i++)
		{
			memcpy(data, pFile, sizeof(float) * 3);
			pFile += pt_length;
			pcl::PointXYZ pt;
			pt.x = data[0];
			pt.y = data[1];
			pt.z = data[2];
			cloud->push_back(pt);
		}
		delete[]data;
	}

	template <>
	inline void parseSPTmmf <PointXYZINTF>(const char *pFile, const uint32_t pts_num, const pcl::PointCloud<PointXYZINTF>::Ptr& cloud)
	{
		float* data = new float[4];
		uint8_t* propdata = new uint8_t[4];
		double* timedata = new double;
		for (int i = 0; i < pts_num; i++)
		{
			memcpy(data, pFile, sizeof(float) * 4);
			pFile += sizeof(float) * 4;
			memcpy(propdata, pFile, sizeof(uint8_t) * 4);
			pFile += sizeof(uint8_t) * 4;
			memcpy(timedata, pFile, sizeof(double));
			pFile += sizeof(double);
			PointXYZINTF pt;
			pt.x = data[0];
			pt.y = data[1];
			pt.z = data[2];
			pt.intensity = data[3];
			pt.num_of_returns = propdata[0];
			pt.return_number = propdata[1];
			pt.flighting_line_edge = propdata[2];
			pt.classification = propdata[3];
			pt.gps_time = timedata[0];
			cloud->push_back(pt);
		}
		delete[] data;
		delete[] timedata;
		delete[] propdata;
	}


	template <typename T>
	bool loadSPT(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& offset)
	{
		if (!std::is_same<T, pcl::PointXYZ>::value && !std::is_same<T, PointXYZINTF>::value)
		{
			LOG(WARNING) << "Warning: skip parsing when point type isn't 'PointXYZ' or 'PointXYZINTF'!";
			return false;
		}
		if (!file_exist(filename))
		{
			LOG(ERROR) << "'" << filename << "' doesn't exist!";
			return false;
		}
		if (is_directory(filename))
		{
			LOG(ERROR) << "'" << filename << "' is a directory!";
			return false;
		}
		if (cloud == nullptr)
		{
			LOG(ERROR) << "pointer 'cloud' is nullptr!";
			return false;
		}
		// check extension
		std::string ext = boost::filesystem::extension(filename);
		if (ext.compare(".spt"))
			LOG(ERROR) << "It's a inappropriate file format.";

		// simgle point length,x,y,z,intensity,gpstime,number_of_returns,return_number,classification and edge_of_flight
		uint32_t pt_length = sizeof(float) * 4 + sizeof(double) + sizeof(uint8_t) * 4;
		// header offset which is the three double's byte here in spt file
		uint32_t spt_offset = sizeof(double) * 3;
		try
		{
			// file size
			uintmax_t sz = boost::filesystem::file_size(filename);
			if (sz >= 300 * 1024 * 1024) //larger than 300M
			{
				HANDLE file_handle = CreateFile(filename.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
				if (INVALID_HANDLE_VALUE == file_handle)
				{
					LOG(ERROR) << "Failed to create a mmapping file.";
					return false;
				}
				// a signed 64 bit integer who is a union including DWORD lowpart, LONG highpart and a LONGLONG QuadPart
				LARGE_INTEGER file_sz;
				// get file size and return to a LARGE_INTEGER
				GetFileSizeEx(file_handle, &file_sz);
				HANDLE mapping_handle = CreateFileMapping(file_handle, NULL, PAGE_READONLY, 0, 0, "SPT FILE MAPPING");
				if (INVALID_HANDLE_VALUE == mapping_handle)
				{
					LOG(ERROR) << "Mapping file failed.";
					return false;
				}
				// error check
				if (GetLastError() == ERROR_FILE_INVALID)
				{
					LOG(ERROR) << "Fail to create a mmapping which is zero.";
				}
				else if (GetLastError() == ERROR_INVALID_HANDLE)
				{
					LOG(ERROR) << "The mmapping's lpName is repeated once. ";
				}
				else if (GetLastError() == ERROR_ALREADY_EXISTS)
				{
					LOG(ERROR) << "The mmapping memory space is existed.";
				}
				// system info
				SYSTEM_INFO sys_info;
				GetSystemInfo(&sys_info);
				DWORD processCoreNum = sys_info.dwNumberOfProcessors;
				//virtual memory space's granularity
				DWORD allocation_granularity = sys_info.dwAllocationGranularity;
				//processor info
				DWORD ProcessorType = sys_info.dwProcessorType;
				WORD ProcessorLevel = sys_info.wProcessorLevel;
				WORD ProcessorRevision = sys_info.wProcessorRevision;
				LARGE_INTEGER cur_size;
				cur_size.QuadPart = 0;
				const uint32_t EACH_POINT_NUM = allocation_granularity * 100;
				// should not be too large
				const uint32_t EACH_SIZE = EACH_POINT_NUM * pt_length;
				// num of pts = verified
				int num_pts = (file_sz.QuadPart - 3 * sizeof(double)) / (sizeof(float) * 4 + sizeof(double) + sizeof(uint8_t) * 4); //sizeof ( uint8_t )=1
				//std::cout << filename << " spt files num_pts: " << num_pts << std::endl;
				char *pFile = nullptr;
				// MapViewOfFile return a start pointer position pointing to the file mmapping memory space (memory size is EACH_SIZE * 1.1 or rest_sz)
				// mmapping will divided into several subpieces, subpieces length is rest_sz or EACH_SIZE * 1.1
				pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, 0, 0, EACH_SIZE * 1.1);
				if (pFile == NULL)
				{
					LOG(ERROR) << "Mapping failed";
				}
				// get spt offset
				double* offset_data = new double[3];
				// memcpy ( reinterpret_cast<char*>( &offset_data ), pFile, spt_offset );
				memcpy(offset_data, pFile, spt_offset);
				offset.x = offset_data[0];
				offset.y = offset_data[1];
				offset.z = offset_data[2];
				// the size created by mmf should be changed with the real point cloud data
				pFile += spt_offset;
				parseSPTmmf<T>(pFile, EACH_POINT_NUM, cloud);
				UnmapViewOfFile(pFile - spt_offset);
				cur_size.QuadPart += EACH_SIZE;
				while (true)
				{
					if (cur_size.QuadPart + EACH_SIZE * 1.1 < file_sz.QuadPart)
					{
						pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, cur_size.HighPart, cur_size.LowPart, EACH_SIZE * 1.1);
						pFile += spt_offset;
						parseSPTmmf<T>(pFile, EACH_POINT_NUM, cloud);
						UnmapViewOfFile(pFile - spt_offset);
						cur_size.QuadPart += EACH_SIZE;
					}
					else //the last section
					{
						uint32_t rest_sz = file_sz.QuadPart - cur_size.QuadPart;
						pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, cur_size.HighPart, cur_size.LowPart, rest_sz);
						pFile += spt_offset;
						uint32_t rest_count = num_pts - cur_size.QuadPart / pt_length;
						parseSPTmmf<T>(pFile, rest_count, cloud);
						UnmapViewOfFile(pFile - spt_offset);
						break;
					}
				}
				CloseHandle(mapping_handle);
				CloseHandle(file_handle);
				delete[]offset_data;
			}
			else //less than 300M
			{
				parseSPT<T>(filename, cloud, offset);
			}
		}
		catch (std::bad_alloc* e)
		{
			LOG(ERROR) << "Error occurred when parsing spt file: " << e->what();
		}
		catch (std::bad_exception* e)
		{
			LOG(ERROR) << "Error occurred when parsing spt file: " << e->what();
		}
		catch (std::exception* e)
		{
			LOG(ERROR) << "Error occurred when parsing spt file: " << e->what();
		}
		return true;
	}

	// the red one is fixed while the green one is src or src_refined
	template <typename T>
	void saveReg(const std::string &filename,
		const typename pcl::PointCloud<T>::Ptr& src_cloud,
		const typename pcl::PointCloud<T>::Ptr& tar_cloud,
		const Offset& offset)
	{
	}

	template <>
	inline void saveReg <pcl::PointXYZ>(const std::string &filename,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& tar_cloud,
		const Offset& offset)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr reg_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		for (auto& pt : src_cloud->points)
		{
			pcl::PointXYZRGB color_pt;
			color_pt.x = pt.x;
			color_pt.y = pt.y;
			color_pt.z = pt.z;
			color_pt.r = 0;
			color_pt.g = 255;
			color_pt.b = 0;
			reg_cloud->push_back(color_pt);
		}
		for (auto& pt : tar_cloud->points)
		{
			pcl::PointXYZRGB color_pt;
			color_pt.x = pt.x;
			color_pt.y = pt.y;
			color_pt.z = pt.z;
			color_pt.r = 255;
			color_pt.g = 0;
			color_pt.b = 0;
			reg_cloud->push_back(color_pt);
		}
		PointIO::saveLAS<pcl::PointXYZRGB>(filename, reg_cloud, offset);
	}

	template <>
	inline void saveReg <PointXYZINTF>(const std::string &filename,
		const pcl::PointCloud<PointXYZINTF>::Ptr &src_cloud,
		const pcl::PointCloud<PointXYZINTF>::Ptr &tar_cloud,
		const Offset& offset)
	{
		pcl::PointCloud<PointXYZRGBINTF>::Ptr reg_cloud = boost::make_shared<pcl::PointCloud<PointXYZRGBINTF>>();
		for (auto& pt : src_cloud->points)
		{
			PointXYZRGBINTF color_pt;
			color_pt.x = pt.x;
			color_pt.y = pt.y;
			color_pt.z = pt.z;
			color_pt.r = 0;
			color_pt.g = 255;
			color_pt.b = 0;
			color_pt.intensity = pt.intensity;
			color_pt.num_of_returns = pt.num_of_returns;
			color_pt.return_number = pt.return_number;
			color_pt.classification = pt.classification;
			color_pt.gps_time = pt.gps_time;
			color_pt.flighting_line_edge = pt.flighting_line_edge;
			reg_cloud->push_back(color_pt);
		}
		for (auto& pt : tar_cloud->points)
		{
			PointXYZRGBINTF color_pt;
			color_pt.x = pt.x;
			color_pt.y = pt.y;
			color_pt.z = pt.z;
			color_pt.r = 255;
			color_pt.g = 0;
			color_pt.b = 0;
			color_pt.intensity = pt.intensity;
			color_pt.num_of_returns = pt.num_of_returns;
			color_pt.return_number = pt.return_number;
			color_pt.classification = pt.classification;
			color_pt.gps_time = pt.gps_time;
			color_pt.flighting_line_edge = pt.flighting_line_edge;
			reg_cloud->push_back(color_pt);
		}
		PointIO::saveLAS<PointXYZRGBINTF>(filename, reg_cloud, offset);
	}

	template <typename T>
	void outputRegPointCloud(const std::string& output_filename, const typename pcl::PointCloud<T>::Ptr& src_cloud,
		const typename pcl::PointCloud<T>::Ptr& tar_cloud, const Eigen::Matrix4f& src2tar, const Utility::Offset& offset)
	{
		pcl::PointCloud<T>::Ptr src_refined_cloud = boost::make_shared<pcl::PointCloud<T>>();
		if (src2tar != Eigen::Matrix4f::Identity())
			pcl::transformPointCloud(*src_cloud, *src_refined_cloud, src2tar);
		else
			src_refined_cloud = src_cloud;
		saveReg <T>(output_filename, src_refined_cloud, tar_cloud, offset);
	}

	// FOR ESTONIA LEASER SCANNING TXT FILES
	// parse txt file
	template <typename T>
	void parseTXT(const std::string& filename, std::ifstream &traj_file, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& las_offset)
	{
	}

	template <>
	inline void parseTXT<PointXYZINTF>(const std::string& filename, std::ifstream &infile, const pcl::PointCloud<PointXYZINTF>::Ptr& cloud, Offset& las_offset)
	{
		int i = 0;
		while (!infile.eof())
		{
			if (i < 1)
			{
				std::string s;
				std::getline(infile, s);
				++i;
				continue;
			}
			std::string singlelinestring, tmp;
			double pointtimpstamp;
			infile >> singlelinestring;
			std::vector<std::string> datavec;
			std::stringstream input(singlelinestring);
			while (getline(input, tmp, ',')) datavec.emplace_back(tmp);
			if (datavec.size() > 1) {
				if (i == 1)
				{
					las_offset = Offset(std::floor(stod(datavec[0])), std::floor(stod(datavec[1])), std::floor(stod(datavec[2])));
					// is this right? due to the txt file i can't centerilized all the point
				}
				PointXYZINTF pt;
				pt.x = stod(datavec[0]) - las_offset.x;
				pt.y = stod(datavec[1]) - las_offset.y;
				pt.z = stod(datavec[2]) - las_offset.z;
				pt.intensity = stof(datavec[3]);
				pt.num_of_returns = 1;
				pt.return_number = 1;
				pt.classification = 1;
				pointtimpstamp = stod(datavec[4]) - 1539413300;
				pointtimpstamp *= 1000000;
				pointtimpstamp = static_cast<double>(pointtimpstamp) / 1000000.0;
				pt.gps_time = pointtimpstamp;
				pt.flighting_line_edge = 1;
				cloud->push_back(pt);
			}
			else
			{
				LOG(WARNING) << "The traj file hasn't parsed correctly! " << filename;
				break;
			}
			++i;
		}
	}

	template <typename T>
	bool loadSingleTXT(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& las_offset)
	{
		if (!file_exist(filename))
		{
			LOG(ERROR) << filename << " doesn't exist!";
			return false;
		}
		if (is_directory(filename))
		{
			LOG(ERROR) << filename << " is a directory!";
			return false;
		}
		if (cloud == nullptr)
		{
			LOG(ERROR) << "pointer cloud is nullptr!" << filename;
			return false;
		}
		try
		{
			// open file
			std::ifstream ifs;
			ifs.open(filename, std::ios::in | std::ios::binary);
			if (!ifs)
			{
				LOG(ERROR) << "Point cloud file stream is incoorectly" << filename;
				return false;
			}
			// file size
			uintmax_t sz = boost::filesystem::file_size(filename);
			if (sz >= 400 * 1024 * 1024)
				LOG(WARNING) << "Currently this way of load txt point file is not so efficient enough" << filename;

			// read Estonia txt file
			parseTXT<T>(filename, ifs, cloud, las_offset);

			ifs.close();
		}
		catch (std::exception* e)
		{
			LOG(ERROR) << "Error occurred when parsing txt file: " << e->what();
		}
		return true;
	}
} // namespace PointIO

#endif // TEMPLATE_POINTCLOUD_IO