
// Local
#include "dbscan.h"
#include "PointCloudIO.h"

namespace Util
{
	void DBSCAN::setInputCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
	{
		ClusterPoint p;
		for (size_t i = 0; i < cloud.points.size(); ++i)
		{
			p.x = cloud.points[i].x;
			p.y = cloud.points[i].y;
			p.z = cloud.points[i].z;
			p.cluster_id = -1; // cluster unknown
			m_Points.push_back(p);
		}
	}

	void DBSCAN::getIDX(std::vector<int> &idx)
	{
		idx.resize(m_Points.size());
		for (unsigned int i = 0; i < m_Points.size(); ++i)
		{
			idx[i] = m_Points[i].cluster_id;
		}
	}

	int DBSCAN::getClusterNumber()
	{
		return m_ClusterNums;
	}

	void DBSCAN::clustering()
	{
		if (m_Points.size() < 2)
		{
			return;
		}
		// cluster id is larger than 1
		int cur_cluster_id = 1;
		std::vector<ClusterPoint>::iterator iter;
		for (iter = m_Points.begin(); iter != m_Points.end(); ++iter)
		{
			if (iter->cluster_id == -1) // if a new point w/o clustered
			{ // if unclustered, assign a id to it
				if (expandCluster(*iter, cur_cluster_id))
				{
					cur_cluster_id++;
				}
			}
		}
		// remove 0 id
		m_ClusterNums = cur_cluster_id - 1;
	}

	bool DBSCAN::expandCluster(ClusterPoint &point, const int &assign_id)
	{
		// fetch the neighborhood points
		std::vector<int> clusterSeeds = calculateCluster(point);
		if (clusterSeeds.size() < m_MinPts)
		{
			point.cluster_id = 0; // noise
			return false;
		}
		else
		{
			int index = 0, indexCorePoint = 0;
			std::vector<int>::iterator iterSeeds;
			for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
			{
				m_Points.at(*iterSeeds).cluster_id = assign_id;
				if (m_Points.at(*iterSeeds).x == point.x &&
					m_Points.at(*iterSeeds).y == point.y &&
					m_Points.at(*iterSeeds).z == point.z)
				{
					indexCorePoint = index;
				}
				++index;
			}
			// erase the seed point and retaib others
			clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);
			// find the neighborhood of retain seed points recurtively
			for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
			{
				std::vector<int> clusterNeighors = calculateCluster(m_Points.at(clusterSeeds[i]));
				if (clusterNeighors.size() < m_MinPts) continue;
				std::vector<int>::iterator iterNeighors;
				for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
				{
					if (m_Points.at(*iterNeighors).cluster_id == -1 ||
						m_Points.at(*iterNeighors).cluster_id == 0)
					{
						if (m_Points.at(*iterNeighors).cluster_id == -1)
						{
							clusterSeeds.push_back(*iterNeighors);
							n = clusterSeeds.size();
						}
						m_Points.at(*iterNeighors).cluster_id = assign_id;
					}
				}
			}
			return true;
		}
	}

	std::vector<int> DBSCAN::calculateCluster(ClusterPoint &point)
	{
		int index = 0;
		std::vector<ClusterPoint>::iterator iter;
		std::vector<int> clusterIndex;
		double distThreshold = m_Eps * m_Eps;
		for (iter = m_Points.begin(); iter != m_Points.end(); ++iter)
		{
			if (calculateDistance(point, *iter) <= distThreshold)
			{
				clusterIndex.push_back(index);
			}
			index++;
		}
		return clusterIndex;
	}

	inline double DBSCAN::calculateDistance(ClusterPoint &pointCore, ClusterPoint &pointTarget)
	{
		float dx = pointCore.x - pointTarget.x;
		float dy = pointCore.y - pointTarget.y;
		float dz = pointCore.z - pointTarget.z;
		return (dx*dx + dy*dy + dz*dz);
	}

	void DBSCAN::print()
	{
		printf("============DBSCAN==========\r\n");
		printf("Number of points: %u\r\n", static_cast<int>(m_Points.size()));
		printf("x     y     z     cluster_id\r\n");
		printf("----------------------------\r\n");
		for (unsigned int i = 0; i < m_Points.size(); i++)
		{
			printf("%6.3lf %6.3lf %6.3lf: %d\r\n",
				m_Points[i].x,
				m_Points[i].y,
				m_Points[i].z,
				m_Points[i].cluster_id);
		}
		printf("=============================\r\n");
	}

	int DBSCAN::save2files(const std::string &outputfile)
	{
		pcl::PointCloud<pcl::PointXYZL>::Ptr out_pt_cloud(new pcl::PointCloud<pcl::PointXYZL>());
		for (auto pt:m_Points)
		{
			pcl::PointXYZL out_pt;
			out_pt.x = pt.x;
			out_pt.y = pt.y;
			out_pt.z = pt.z;
			out_pt.label = pt.cluster_id;
			out_pt_cloud->push_back(out_pt);
		}
		PointIO::savePCD<pcl::PointXYZL>(outputfile, out_pt_cloud);
	}

	int DBSCAN::getBiggestCluster()
	{
		int *hist = new int[m_ClusterNums + 1]();
		// traverse each point
		for (size_t i = 0; i < m_Points.size(); i++)
		{
			hist[m_Points[i].cluster_id]++;
		}
		int count = 0;
		int maxid;
		for (int k = 1; k <= m_ClusterNums; k++)
		{
			if (hist[k] > count)
			{
				count = hist[k];
				maxid = k;
			}
		}
		// release memory
		delete[] hist;
		hist = NULL;
		return maxid;
	}
}