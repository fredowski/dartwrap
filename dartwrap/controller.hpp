/* mlhole - a small example for modelling a nail and a hole
   Copyright (c) 2016, Friedrich Beckmann, Hochschule Augsburg

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef MLHOLE_CONTROLLER
#define MLHOLE_CONTROLLER

#include <Eigen/Eigen>
#include "dart/dart.hpp"

/// \brief 
class Controller
{
public:
  /// \brief Constructor
  Controller(dart::dynamics::SkeletonPtr _stick);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief
  void update(const Eigen::Vector6d& _targetPosition);

private:
  // The stick is expected to be composed of a freejoint and
  // one body.
  dart::dynamics::SkeletonPtr mStick;

  // The forces which are applied to the joint
  Eigen::VectorXd mForces;

  // The P(roportional) factors for the control scheme
  Eigen::DiagonalMatrix<double,6> mKp;
  Eigen::DiagonalMatrix<double,6> mKd;

};


#endif /* MLHOLE_CONTROLLER */
