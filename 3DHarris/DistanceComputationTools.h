//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef DISTANCE_COMPUTATION_TOOLS_HEADER
#define DISTANCE_COMPUTATION_TOOLS_HEADER

//Local
#include "CCConst.h"
#include "CCToolbox.h"
#include "CCTypes.h"
#include "CCGeom.h"
#include "ScalarField.h"
#include "GenericCloud.h"

namespace CCLib
{

class GenericCloud;
class GenericIndexedCloudPersist;
class PointCloud;
class ScalarField;

//! Several entity-to-entity distances computation algorithms (cloud-cloud, cloud-mesh, point-triangle, etc.)
class DistanceComputationTools : public CCToolbox
{
public: //distance to simple entities (triangles, planes, etc.)

	//! Computes the (signed) distance between a point and a plane
	/** \param P a 3D point
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d' with norm(a,bc)==1
		\return the signed distance between the point and the plane
	**/
	static ScalarType computePoint2PlaneDistance(const CCVector3* P, const PointCoordinateType* planeEquation);

	//! Error estimators
	enum ERROR_MEASURES
	{
		RMS,						/**< Root Mean Square error **/
		MAX_DIST_68_PERCENT,		/**< Max distance @ 68% (1 sigma) **/
		MAX_DIST_95_PERCENT,		/**< Max distance @ 98% (2 sigmas) **/
		MAX_DIST_99_PERCENT,		/**< Max distance @ 99% (3 sigmas) **/
		MAX_DIST,					/**< Max distance **/
	};

	//! Computes the "distance" (see ERROR_MEASURES) between a point cloud and a plane
	/** \param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\param measureType measure type
	**/
	static ScalarType ComputeCloud2PlaneDistance(	CCLib::GenericCloud* cloud,
													const PointCoordinateType* planeEquation,
													ERROR_MEASURES measureType);

	//! Computes the maximum distance between a point cloud and a plane
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\param percent percentage of lowest values ignored
		\return the max distance @ 'percent' % between the point and the plane
	**/
	static ScalarType ComputeCloud2PlaneRobustMax(	GenericCloud* cloud,
													const PointCoordinateType* planeEquation,
													float percent);

	//! Computes the maximum distance between a point cloud and a plane
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\return the max distance between the point and the plane
	**/
	static ScalarType ComputeCloud2PlaneMaxDistance(GenericCloud* cloud,
													const PointCoordinateType* planeEquation);

	//! Computes the Root Mean Square (RMS) distance between a cloud and a plane
	/** Sums the squared distances between each point of the cloud and the plane, then computes the mean value.
		WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param planeEquation plane equation: [a,b,c,d] as 'ax+by+cz=d'
		\return the RMS of distances (or NaN if an error occurred)
	**/
	static ScalarType computeCloud2PlaneDistanceRMS(	GenericCloud* cloud,
														const PointCoordinateType* planeEquation);
};

}

#endif //DISTANCE_COMPUTATION_TOOLS_HEADER
