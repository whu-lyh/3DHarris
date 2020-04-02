#pragma once
#ifdef TEMPLATE_POINTCLOUD_IO

#include "basicLibs.h"
#include "Utility.h"
#include "PointCloudIO.h"
#include "PointType.h"

using namespace Utility;

namespace PointIO
{
	//parse las file less than 400M
	template <typename T>
	void parseLAS(liblas::Reader& reader, const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
	{
	}

	template <>
	inline void parseLAS<PointXYZINTF>(liblas::Reader& reader, const pcl::PointCloud<PointXYZINTF>::Ptr& cloud,const Offset& offset)
	{
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			PointXYZINTF pt;
			pt.x = p.GetX()- offset.x;
			pt.y = p.GetY()- offset.y;
			pt.z = p.GetZ()- offset.z;
			pt.intensity = p.GetIntensity();
			pt.num_returns = p.GetNumberOfReturns();
			pt.gps_time = p.GetTime();
			pt.flighting_line_edge = p.GetFlightLineEdge();
			cloud->push_back(pt);
		}
	}

	template <>
	inline void parseLAS<pcl::PointXYZ>(liblas::Reader& reader, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Offset& offset)
	{
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			pcl::PointXYZ pt;
			pt.x = p.GetX() - offset.x;
			pt.y = p.GetY() - offset.y;
			pt.z = p.GetZ() - offset.z;
			cloud->push_back (pt);
		}
	}

	template <>
	inline void parseLAS<pcl::PointXYZRGB>(liblas::Reader& reader, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Offset& offset)
	{
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			pcl::PointXYZRGB pt;
			pt.x = p.GetX() - offset.x;
			pt.y = p.GetY() - offset.y;
			pt.z = p.GetZ() - offset.z;
			pt.r = p.GetColor().GetRed() >> 8;
			pt.g = p.GetColor().GetGreen() >> 8;
			pt.b = p.GetColor().GetBlue() >> 8;

			cloud->push_back (pt);
		}
	}

	//parse las file larger than 400M
	template <typename T>
	void parseLASmmf(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
		const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
	{}

	template <>
	inline void parseLASmmf<PointXYZINTF>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
		const pcl::PointCloud<PointXYZINTF>::Ptr& cloud, const Offset& offset)
	{
		switch (header.GetDataFormatId())
		{
			case liblas::ePointFormat0:
			case liblas::ePointFormat1:
			case liblas::ePointFormat2:
			case liblas::ePointFormat3:
			{
				for (int i = 0; i < pts_num; i++)
				{
					liblas::Point p(&header);
					memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
					LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
					PointXYZINTF pt;
					pt.x = p.GetX()- offset.x;
					pt.y = p.GetY()- offset.y;
					pt.z = p.GetZ()- offset.z;
					pt.intensity = p.GetIntensity();
					pt.num_returns = p.GetNumberOfReturns();
					pt.gps_time = p.GetTime();
					pt.flighting_line_edge = p.GetFlightLineEdge();
					cloud->push_back (pt);
				}
				break;
			}
			default:
			{
				std::cout << "暂不支持Point Data Record Format 4及以上";
				return;
			}
		}
	}

	template <>
	inline void parseLASmmf<pcl::PointXYZ>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Offset& offset)
	{
		switch (header.GetDataFormatId())
		{
		case liblas::ePointFormat0:
		case liblas::ePointFormat1:
		case liblas::ePointFormat2:
		case liblas::ePointFormat3:
		{
			for (int i = 0; i < pts_num; i++)
			{
				liblas::Point p(&header);
				memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
				LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
				pcl::PointXYZ pt;
				pt.x = p.GetX() - offset.x;
				pt.y = p.GetY() - offset.y;
				pt.z = p.GetZ() - offset.z;
				cloud->push_back (pt);
			}
			break;
		}
		default:
		{
			std::cout << "暂不支持Point Data Record Format 4及以上";
			return;
		}
		}
	}

	template <>
	inline void parseLASmmf<pcl::PointXYZRGB>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Offset& offset)
	{
		switch (header.GetDataFormatId())
		{
		case liblas::ePointFormat0:
		case liblas::ePointFormat1:
		case liblas::ePointFormat2:
		case liblas::ePointFormat3:
		{
			for (int i = 0; i < pts_num; i++)
			{
				liblas::Point p(&header);
				memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
				LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
				pcl::PointXYZRGB pt;
				pt.x = p.GetX() - offset.x;
				pt.y = p.GetY() - offset.y;
				pt.z = p.GetZ() - offset.z;
				pt.r = p.GetColor().GetRed() >> 8;
				pt.g = p.GetColor().GetGreen() >> 8;
				pt.b = p.GetColor().GetBlue() >> 8;
				cloud->push_back (pt);
			}
			break;
		}
		default:
		{
			std::cout << "暂不支持Point Data Record Format 4及以上";
			return;
		}
		}
	}

	template <typename T>
	bool loadSingleLAS(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& las_offset)
	{
		if (!file_exist(filename))
		{
			std::cout << "'" << filename << "' doesn't exist!";
			return false;
		}

		if (is_directory(filename))
		{
			std::cout << "'" << filename << "' is a directory!";
			return false;
		}

		if (cloud == nullptr)
		{
			std::cout << "pointer 'cloud' is nullptr!";
			return false;
		}

		std::cout << "Load file: " << filename;
		try
		{
			//check extension
			std::string ext = boost::filesystem::extension(filename);
			if (ext.compare(".las"))
				std::cout << "It's a inappropriate file format.";

			//open file
			std::ifstream ifs;
			ifs.open(filename, std::ios::in | std::ios::binary);
			if (!ifs)
				return false;

			liblas::ReaderFactory f;
			liblas::Reader reader = f.CreateWithStream(ifs);
			const liblas::Header header = reader.GetHeader();
			uint8_t major_version = header.GetVersionMajor();
			uint8_t minor_version = header.GetVersionMinor();
			if (major_version > 1 || minor_version > 3)
			{
				std::cout << "Currently this app doesn't support version newer than 1.4";
				return false;
			}

			//num of points
			uint32_t pts_count = header.GetPointRecordsCount();
			uint32_t offset = header.GetDataOffset();
			uint32_t pt_length = header.GetDataRecordLength();
			liblas::PointFormatName las_format = header.GetDataFormatId();
			double offset_x = header.GetOffsetX();
			double offset_y = header.GetOffsetY();
			double offset_z = header.GetOffsetZ();
			las_offset = Offset(offset_x, offset_y, offset_z);

			//bounding box
			liblas::Bounds<double> bound = header.GetExtent();
			if (bound.empty())
			{
				std::cout << "The header of this las doesn't contain extent. The cache cannot be built.";
				return false;
			}

			//file size
			uintmax_t sz = boost::filesystem::file_size(filename);

			if (sz >= 400 * 1024 * 1024) //larger than 400M
			{
				HANDLE file_handle = CreateFile(filename.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
					FILE_ATTRIBUTE_NORMAL, NULL);
				if (INVALID_HANDLE_VALUE == file_handle)
				{
					std::cout << "Failed because of err.";
					return false;
				}

				LARGE_INTEGER file_sz;
				GetFileSizeEx(file_handle, &file_sz);
				HANDLE mapping_handle = CreateFileMapping(file_handle, NULL, PAGE_READONLY, 0, 0, "LAS FILE MAPPING");
				if (INVALID_HANDLE_VALUE == mapping_handle)
				{
					std::cout << "Mapping file failed.";
					return false;
				}

				//system info
				SYSTEM_INFO sys_info;
				GetSystemInfo(&sys_info);
				DWORD allocation_granularity = sys_info.dwAllocationGranularity;

				LARGE_INTEGER cur_size;
				cur_size.QuadPart = 0;
				const uint32_t EACH_POINT_NUM = allocation_granularity * 200;
				const uint32_t EACH_SIZE = EACH_POINT_NUM * pt_length;
				char *pFile = nullptr;
				pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, 0, 0, EACH_SIZE * 1.1);

				pFile += offset;
				parseLASmmf<T>(pFile, EACH_POINT_NUM, header, pt_length, cloud, las_offset);
				UnmapViewOfFile(pFile - offset);
				cur_size.QuadPart += EACH_SIZE;
				while (true)
				{
					if (cur_size.QuadPart + EACH_SIZE * 1.1 < file_sz.QuadPart)
					{
						pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, cur_size.HighPart, cur_size.LowPart, EACH_SIZE * 1.1);
						pFile += offset;
						parseLASmmf<T>(pFile, EACH_POINT_NUM, header, pt_length, cloud, las_offset);
						UnmapViewOfFile(pFile - offset);
						cur_size.QuadPart += EACH_SIZE;
					}
					else //the last section
					{
						uint32_t rest_sz = file_sz.QuadPart - cur_size.QuadPart;
						pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, cur_size.HighPart, cur_size.LowPart, rest_sz);
						pFile += offset;
						uint32_t rest_count = pts_count - cur_size.QuadPart / pt_length;
						parseLASmmf<T>(pFile, rest_count, header, pt_length, cloud, las_offset);
						UnmapViewOfFile(pFile);
						break;
					}
				}
				CloseHandle(mapping_handle);
				CloseHandle(file_handle);
			}
			else //less than 400M
			{
				parseLAS<T>(reader, cloud, las_offset);
			}

			ifs.close();
		}
		catch (std::exception* e)
		{
			std::cout << "Error occurred when parsing las file: " << e->what();
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
	inline bool saveLAS2<pcl::PointXYZRGB>(const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			std::cout << "Pointer 'cloud' is nullptr!";
			return false;
		}

		if (cloud->empty())
		{
			std::cout << "Point cloud is empty!";
			return false;
		}

		std::ofstream ofs;
		ofs.open(filepath, std::ios::out | std::ios::binary);
		ofs.setf(std::ios::fixed, std::ios::floatfield);
		ofs.precision(6);

		if (ofs.is_open())
		{
			liblas::Header header;
			header.SetDataFormatId(liblas::ePointFormat2);
			header.SetVersionMajor(1);
			header.SetVersionMinor(2);
			header.SetOffset(offset.x, offset.y, offset.z);
			header.SetScale(0.0001, 0.0001, 0.0001);
			header.SetPointRecordsCount(cloud->size());

			liblas::Writer writer(ofs, header);
			liblas::Point pt(&header);

			for (int i = 0; i < cloud->size(); ++i)
			{
				double x = static_cast<double>(cloud->points[i].x) + offset.x;
				double y = static_cast<double>(cloud->points[i].y) + offset.y;
				double z = static_cast<double>(cloud->points[i].z) + offset.z;

				pt.SetCoordinates(x, y, z);
				pt.SetIntensity(10);
				pt.SetColor(liblas::Color(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b));

				writer.WritePoint(pt);
			}
			ofs.flush();
			ofs.close();
		}
		
		std::cout << "Save file: " << filepath;
		return true;
	}

	template <>
	inline bool saveLAS2<pcl::PointXYZ>(const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			std::cout << "Pointer 'cloud' is nullptr!";
			return false;
		}

		if (cloud->empty())
		{
			std::cout << "Point cloud is empty!";
			return false;
		}

		std::ofstream ofs;
		ofs.open(filepath, std::ios::out | std::ios::binary);
		ofs.setf(std::ios::fixed, std::ios::floatfield);
		ofs.precision(6);

		if (ofs.is_open())
		{
			liblas::Header header;
			header.SetDataFormatId(liblas::ePointFormat2);
			header.SetVersionMajor(1);
			header.SetVersionMinor(2);
			header.SetOffset(offset.x, offset.y, offset.z);
			header.SetScale(0.0001, 0.0001, 0.0001);
			header.SetPointRecordsCount(cloud->size());

			liblas::Writer writer(ofs, header);
			liblas::Point pt(&header);

			for (int i = 0; i < cloud->size(); ++i)
			{
				double x = static_cast<double>(cloud->points[i].x) + offset.x;
				double y = static_cast<double>(cloud->points[i].y) + offset.y;
				double z = static_cast<double>(cloud->points[i].z) + offset.z;

				pt.SetCoordinates(x, y, z);
				pt.SetIntensity(10);

				writer.WritePoint(pt);
			}
			ofs.flush();
			ofs.close();
		}

		std::cout << "Save file: " << filepath;
		return true;
	}

	template <>
	inline bool saveLAS2<pcl::PointXYZI> ( const std::string& filepath, const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
										  const Offset& offset )
	{
		if ( cloud == nullptr )
		{
			std::cout << "Pointer 'cloud' is nullptr!";
			return false;
		}

		if ( cloud->empty () )
		{
			std::cout << "Point cloud is empty!";
			return false;
		}

		std::ofstream ofs;
		ofs.open ( filepath, std::ios::out | std::ios::binary );
		ofs.setf ( std::ios::fixed, std::ios::floatfield );
		ofs.precision ( 6 );

		if ( ofs.is_open () )
		{
			liblas::Header header;
			header.SetDataFormatId ( liblas::ePointFormat2 );
			header.SetVersionMajor ( 1 );
			header.SetVersionMinor ( 2 );
			header.SetOffset ( offset.x, offset.y, offset.z );
			header.SetScale ( 0.0001, 0.0001, 0.0001 );
			header.SetPointRecordsCount ( cloud->size () );

			liblas::Writer writer ( ofs, header );
			liblas::Point pt ( &header );

			for ( int i = 0; i < cloud->size (); ++i )
			{
				double x = static_cast<double>( cloud->points [i].x ) + offset.x;
				double y = static_cast<double>( cloud->points [i].y ) + offset.y;
				double z = static_cast<double>( cloud->points [i].z ) + offset.z;

				uint16_t intest = static_cast<uint16_t>( cloud->points [i].intensity );

				pt.SetCoordinates ( x, y, z );
				pt.SetIntensity ( intest );

				writer.WritePoint ( pt );
			}
			ofs.flush ();
			ofs.close ();
		}

		std::cout << "Save file: " << filepath;
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
			std::cout << "pointer 'cloud' is nullptr!";
			return false;
		}

		if (pcl::io::loadPCDFile(filename, *cloud) != -1)
		{
			std::cout << "Load PCD file: '" << filename << "'";
			return true;
		}
		return false;
	}

	template <typename T>
	bool savePCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud,
		const Offset& offset)
	{
		if (cloud == nullptr)
		{
			std::cout << "pointer 'cloud' is nullptr!";
			return false;
		}

		if (cloud->empty())
		{
			std::cout << "point cloud is empty!";
			return false;
		}

		if (std::fabs(offset.x) <= Epsilon_d && std::fabs(offset.y) <= Epsilon_d && std::fabs(offset.z) <= Epsilon_d)
		{
			pcl::io::savePCDFileBinary(filename, *cloud);
		}
		else
		{
			pcl::PointCloud<T> out_cloud;
			for (const auto& pt : cloud->points)
			{
				T out_pt = pt;
				out_pt.x/* += offset.x*/;
				out_pt.y/* += offset.y*/;
				out_pt.z/* += offset.z*/;
				out_cloud.emplace_back(out_pt);
			}

			pcl::io::savePCDFileBinary(filename, out_cloud);
		}

		std::cout << "Save PCD file: '" << filename << "'";
		return true;
	}

	template <typename T>
	bool loadSPT(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Offset& offset)
	{
		if (!std::is_same<T, pcl::PointXYZ>::value)
		{
			LOG(WARNING) << "Warning: skip parsing when point type isn't 'PointXYZ'!";
			return false;
		}

		if (!file_exist(filename))
		{
			std::cout << "'" << filename << "' doesn't exist!";
			return false;
		}

		if (is_directory(filename))
		{
			std::cout << "'" << filename << "' is a directory!";
			return false;
		}

		if (cloud == nullptr)
		{
			std::cout << "pointer 'cloud' is nullptr!";
			return false;
		}

		std::ifstream ifs;
		ifs.open(filename, std::ios::in | std::ios::binary);
		ifs.seekg(0, std::ios::end);

		double* offset_data = new double[3];
		int data_length = ifs.tellg();
		data_length = (data_length -3 * sizeof(double)) / sizeof(float);
		float* data = new float[data_length];


		ifs.seekg(0, std::ios::beg);
		ifs.read(reinterpret_cast<char*>(offset_data), 3 * sizeof(double));
		ifs.seekg(3 * sizeof(double),std::ios::beg);
		ifs.read(reinterpret_cast<char*>(data), data_length * sizeof(float));
		ifs.close();

		offset = Offset(offset_data[0], offset_data[1], offset_data[2]);

		int num_pts = data_length / 3;
		for (int i = 0; i < num_pts; ++i)
		{
			T pt;
			pt.x = data[3 * i];
			pt.y = data[3 * i + 1];
			pt.z = data[3 * i + 2];
			cloud->emplace_back(pt);
		}
		delete[] data;
		delete[] offset_data;

		return true;
	}

	template <typename T>
	void outputRegPointCloud(const std::string& output_filename, const typename pcl::PointCloud<T>::Ptr& src_cloud,
		const typename pcl::PointCloud<T>::Ptr& tar_cloud, const Eigen::Matrix4f& tar2src, const Offset& offset)
	{
		pcl::PointCloud<T>::Ptr tar_refined_cloud = boost::make_shared<pcl::PointCloud<T>>();
		if (tar2src != Eigen::Matrix4f::Identity())
			pcl::transformPointCloud(*tar_cloud, *tar_refined_cloud, tar2src);
		else
			tar_refined_cloud = tar_cloud;
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
			reg_cloud->emplace_back(color_pt);
		}
		for (auto& pt : tar_refined_cloud->points)
		{
			pcl::PointXYZRGB color_pt;
			color_pt.x = pt.x;
			color_pt.y = pt.y;
			color_pt.z = pt.z;
			color_pt.r = 255;
			color_pt.g = 0;
			color_pt.b = 0;
			reg_cloud->emplace_back(color_pt);
		}

		saveLAS<pcl::PointXYZRGB>(output_filename, reg_cloud, offset);
	}
} // namespace PointIO

#endif // TEMPLATE_POINTCLOUD_IO