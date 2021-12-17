// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2017 <Zillow Inc.> Pierre Moulon

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_CAMERAS_CAMERA_SPHERICAL_IO_HPP
#define OPENMVG_CAMERAS_CAMERA_SPHERICAL_IO_HPP

#include "PCImage/numeric/numeric.h"
#include "PCImage/cameras/Camera_Intrinsics.hpp"

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>

template <class Archive>
inline void PCImage::cameras::Intrinsic_Spherical::save( Archive & ar ) const
{
  ar(cereal::base_class<IntrinsicBase>(this));
}

template <class Archive>
inline void PCImage::cameras::Intrinsic_Spherical::load( Archive & ar )
{
  ar(cereal::base_class<IntrinsicBase>(this));
}
CEREAL_REGISTER_TYPE_WITH_NAME(PCImage::cameras::Intrinsic_Spherical, "spherical");

namespace cereal
{
  // This struct specialization will tell cereal which is the right way to serialize the ambiguity
  template <class Archive> struct specialize<Archive, PCImage::cameras::Intrinsic_Spherical, cereal::specialization::member_load_save> {};
}

CEREAL_REGISTER_POLYMORPHIC_RELATION(PCImage::cameras::IntrinsicBase, PCImage::cameras::Intrinsic_Spherical);

#endif // #ifndef OPENMVG_CAMERAS_CAMERA_SPHERICAL_IO_HPP