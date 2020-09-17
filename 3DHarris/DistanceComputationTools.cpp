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

#include "DistanceComputationTools.h"

//local
#include "PointCloud.h"

//system
#include <algorithm>
#include <cassert>

using namespace CCLib;

ScalarType DistanceComputationTools::computePoint2PlaneDistance(const CCVector3* P,
																const PointCoordinateType* planeEquation)
{
	//point to plane distance: d = (a0*x+a1*y+a2*z - a3) / sqrt(a0^2+a1^2+a2^2)
	assert(std::abs(CCVector3::vnorm(planeEquation) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	return static_cast<ScalarType>((CCVector3::vdot(P->u, planeEquation) - planeEquation[3])/*/CCVector3::vnorm(planeEquation)*/); //norm == 1.0!
}

ScalarType DistanceComputationTools::computeCloud2PlaneDistanceRMS(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
    assert(cloud && planeEquation);

	//point count
	unsigned count = cloud->size();
	if (count == 0)
		return 0;

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
        return NAN_VALUE;
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	double dSumSq = 0.0;

	//compute deviations
	cloud->placeIteratorAtBeginning();
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		double d = static_cast<double>(CCVector3::vdot(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0
		
		dSumSq += d*d;
	}

	return static_cast<ScalarType>( sqrt(dSumSq/count) );
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneRobustMax(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation,
																	float percent)
{
    assert(cloud && planeEquation);
	assert(percent < 1.0f);

	//point count
	unsigned count = cloud->size();
	if (count == 0)
		return 0;

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
        return NAN_VALUE;
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	//we search the max @ 'percent'% (to avoid outliers)
	std::vector<PointCoordinateType> tail;
	std::size_t tailSize = static_cast<std::size_t>(ceil(static_cast<float>(count) * percent));
	tail.resize(tailSize);

	//compute deviations
	cloud->placeIteratorAtBeginning();
	std::size_t pos = 0;
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = std::abs(CCVector3::vdot(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0

		if (pos < tailSize)
		{
			tail[pos++] = d;
		}
		else if (tail.back() < d)
		{
			tail.back() = d;
		}

		//search the max element of the tail
		std::size_t maxPos = pos-1;
		if (maxPos != 0)
		{
			std::size_t maxIndex = maxPos;
			for (std::size_t j=0; j<maxPos; ++j)
				if (tail[j] < tail[maxIndex])
					maxIndex = j;
			//and put it to the back!
			if (maxPos != maxIndex)
				std::swap(tail[maxIndex],tail[maxPos]);
		}
	}

	return static_cast<ScalarType>(tail.back());
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneMaxDistance(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
	assert(cloud && planeEquation);

	//point count
	unsigned count = cloud->size();
	if (count == 0)
		return 0;

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
		return NAN_VALUE;
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	//we search the max distance
	PointCoordinateType maxDist = 0;
	
	cloud->placeIteratorAtBeginning();
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = std::abs(CCVector3::vdot(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0
		maxDist = std::max(d,maxDist);
	}

	return static_cast<ScalarType>(maxDist);
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneDistance(CCLib::GenericCloud* cloud,
																const PointCoordinateType* planeEquation,
																ERROR_MEASURES measureType)
{
	switch (measureType)
	{
	case RMS:
		return CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(cloud,planeEquation);

	case MAX_DIST_68_PERCENT:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.32f);
	case MAX_DIST_95_PERCENT:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.05f);
	case MAX_DIST_99_PERCENT:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.01f);
	
	case MAX_DIST:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneMaxDistance(cloud,planeEquation);

	default:
		assert(false);
		return -1.0;
	}
}
