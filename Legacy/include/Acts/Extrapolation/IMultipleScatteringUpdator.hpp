// This file is part of the Acts project.
//
// Copyright (C) 2016 Acts project team
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

///////////////////////////////////////////////////////////////////
// IMultipleScatteringUpdator.h, Acts project
///////////////////////////////////////////////////////////////////

#pragma once
#include "Acts/EventData/ParticleDefinitions.hpp"

namespace Acts {

class MaterialProperties;

// @class IMultipleScatteringUpdator
// Interface class IMultipleScatteringUpdator
class IMultipleScatteringUpdator
{
public:
  /// Virtual destructor
  virtual ~IMultipleScatteringUpdator() {}
  /// Calculate the sigma on theta introduced by multiple scattering
  ///
  /// @param mat are the material properties for this update
  /// @param p is the momentum of the particle
  /// @param pathcorrection is the incident angle correction
  /// @param particle is the particle type for this
  /// @param deltaE is the energy loss associated
  ///
  /// @return a double representing sigma square
  virtual double
  sigmaSquare(const MaterialProperties& mat,
              double                    p,
              double                    pathcorrection,
              ParticleType              particle = pion,
              double                    deltaE = 0.) const = 0;
};

}  // namespace