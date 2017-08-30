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

#include "controller.hpp"

//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _stick)
  : mStick(_stick)
   
{
  assert(_stick != nullptr);

  int dof = mStick->getNumDofs();
  /* This are the control parameters of the controller */
  mKp.diagonal() << 200.0, 200.0, 200.0, 3000.0, 3000.0, 3000.0;
  mKd.diagonal() << 10.0, 10.0, 10.0, 50.0, 50.0, 50.0;
  
  mForces.setZero(dof);

  // Set joint damping
  //for (int i = 0; i < dof; ++i)
  //  _stick->getJoint(0)->setDampingCoefficient(i, 0.5);
}

//==============================================================================
Controller::~Controller()
{
}

//==============================================================================
void Controller::update(const Eigen::Vector6d& _targetPosition)
{
  using namespace dart;

  Eigen::Vector6d cp = mStick->getPositions();
  Eigen::Vector6d cv = mStick->getVelocities();
  dart::dynamics::BodyNodePtr bn = mStick->getBodyNode("cylinder_link");
  Eigen::Isometry3d wt = bn->getWorldTransform();
  /* Normalize the angle positions */
  for(int i=0;i<3;i++)
    if (cp(i) > M_PI)
      cp(i) -= 2.0*M_PI;

  Eigen::Vector6d err = _targetPosition - cp;

  /* Normalize the error angles */
  for(int i=0;i<3;i++)
    {
      if (err(i) > M_PI)
        err(i) -= 2.0*M_PI;
      if (err(i) < -1.0*M_PI)
        err(i) += 2.0*M_PI;
    }

  //std::cout << "Pos: " << cp << std::endl;
  //std::cout << "Vel: " << cv << std::endl;

  /* The velocities are with respect to the body coordinate system.
     These are transformed to the world coordinates. cv.tail(3) are
     the vector velocities. 0-2 are angular velocities */
  cv.tail(3) = wt.linear() * cv.tail(3);
  //std::cout << "Vel trans: " << cv << std::endl;
  //std::cout << "Error: " << err << std::endl;
  /* This is the actual controller */
  mForces = mKp * err - mKd * cv;
  //std::cout << "Forces: " << mForces << std::endl;
  //std::cout << "wt linear: " << wt.linear() << std::endl;
  //std::cout << "wt trans: "  << wt.translation() << std::endl;

  /* The computed forces are transformed back to body coordinates */
  mForces.tail(3) = wt.linear().transpose() * mForces.tail(3);
  //std::cout << "force trans: "  << mForces << std::endl;  
  mStick->getJoint(0)->setForces(mForces);
}
