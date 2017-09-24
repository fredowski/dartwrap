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


#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "controller.hpp"
// glutMainLoopEvent was introduced in freeglut
#include <GL/freeglut.h>
// This are the interface functions to torch/lua
#include "dartwrap.h"

// The stepsize for changing position in x y z direction
const double default_step = 0.01;

// The step angle when changing rotation in Degrees
const double default_angle = 10.0;

using namespace dart::dynamics;
using namespace dart::simulation;

SkeletonPtr createFloor()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr floor =
    loader.parseSkeleton(INSTALL_PREFIX"/data/floor/floor.urdf");
  floor->setName("floor");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

  floor->getJoint("world_joint")->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createStick()
{
  SkeletonPtr stick = Skeleton::create("stick");

  BodyNode::Properties bodyProp;
  bodyProp.mName = "cylinder_link";
  bodyProp.mInertia.setMass(1.0);
  // Do not consider friction - switch to frictionless mode
  bodyProp.mFrictionCoeff = 1e-12;

  FreeJoint::Properties jointProp;
  jointProp.mName = "cylinder_joint";
  
  BodyNodePtr body =
    stick->createJointAndBodyNodePair<FreeJoint>(nullptr, jointProp, bodyProp).second;

  // Switch off gravity for the stick
  body->setGravityMode(false);

  // Give the body a shape
  double radius = 0.005;
  double height = 0.2;
  std::shared_ptr<CylinderShape> cylinder(
                                new CylinderShape(radius, height));
  auto shapeNode
    = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(cylinder);
  shapeNode->getVisualAspect()->setColor(dart::Color::Green());

  // The initial position of the stick
  dart::dynamics::Joint *j = body->getParentJoint();
  j->setPosition(0,0.0*M_PI/180.0);
  j->setPosition(3,0.2);
  j->setPosition(4,0.3);
  j->setPosition(5,0.2);

  return stick;
}

class MyWindow : public dart::gui::SimWindow
{
public:

  /// Constructor
  MyWindow()
    : singlestep(false),
      CamRot(0.5, -0.3, -0.47, -0.67)
  {
    // Create the stick
    SkeletonPtr stick = createStick();

    // Create a world and add the stick to the world
    WorldPtr world(new World);
    world->addSkeleton(stick);

    // Add the floor
    SkeletonPtr floor = createFloor();
    world->addSkeleton(floor);

    setWorld(world);
    
    // Set the initial camera view
    mZoom = 0.8;
    mTrans << 20.0,-63.0,24.0;
    mTrackBall.setQuaternion(CamRot);
    
    // Find the Skeleton named "stick" within the World
    mStick = world->getSkeleton("stick");
    assert(mStick != nullptr);

    mFloor = world->getSkeleton("floor");

    // This is the controller which controls the stick
    mController = new Controller(stick);
    // Initial target position for the stick
    mTargetPosition << 30.0*M_PI/180.0, 0.0, 0.0, 0.2, 0.3, 0.2;

    // Start the simulation
    mSimulating = true;
  }

    /// \brief Destructor
  virtual ~MyWindow(){};

  void drawWorld() const {
    /* Enable filled polygons not just wireframes */
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    SimWindow::drawWorld();
  }

  void initLights() {
    static float ambient[]             = {0.0, 0.0, 0.0, 1.0};
    static float diffuse[]             = {1.0, 1.0, 1.0, 1.0};

    GLfloat position[] = {0.0, 1.0, 2.0, 0.0};

    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
  }
  
  void render()
  {
    if (mShow3D) {
      SimWindow::render();
      return;
    }
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-1.0, 1.0, -1 * 480.0/640.0, 1 * 480.0/640.0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    initGL();

    glScalef(mZoom, mZoom, mZoom);
    glTranslatef(mTrans[0]/640.0*2.0, mTrans[1]/480.0*2.0*48.0/64.0, mTrans[2]/480.0*2.0*48.0/64.0);
    glRotatef(90,-1.0,0,0); // Rotate around x axis to look at pendulum
    initLights();
    draw();
    glutSwapBuffers();
  }

  void click(int _button, int _state, int _x, int _y) {
    if (mShow3D) {
      Win3D::click(_button, _state, _x, _y);
      return;
    }
    mMouseDown = !mMouseDown;
    if (mMouseDown) {
      if (_button == 3 && _state == GLUT_DOWN) {  // mouse wheel up
        // each scroll generates a down and an immediate up,
        // so ignore ups
        mZoom *= 1.1;
      } else if (_button == 4 && _state == GLUT_DOWN) {  // mouse wheel down?
        // each scroll generates a down and an immediate up,
        // so ignore ups
        mZoom *= 0.9;
      }
      mMouseX = _x;
      mMouseY = _y;
    }
    glutPostRedisplay();
  }

  void drag(int _x, int _y) {
    if (mShow3D) {
      Win3D::drag(_x, _y);
      return;
    }
    double deltaX = _x - mMouseX;
    double deltaY = _y - mMouseY;

    mMouseX = _x;
    mMouseY = _y;
    mTrans += Eigen::Vector3d(deltaX / mZoom, -deltaY / mZoom, 0.0);

    glutPostRedisplay();
  }
  
  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
      {
      case 'q':
        mTargetPosition(3) += default_step;
        break;
      case 'a':
        mTargetPosition(3) -= default_step;
        break;

      case 'w':
        mTargetPosition(4) += default_step;
        break;
      case 's':
        mTargetPosition(4) -= default_step;
        break;

      case 'e':
        mTargetPosition(5) += default_step;
        break;
      case 'd':
        mTargetPosition(5) -= default_step;
        break;
      case 'r':
        mTargetPosition(0) += default_angle * M_PI/180.0;
        if (mTargetPosition(0) > M_PI) mTargetPosition(0) -= 2*M_PI;
        break;
      case 'f':
        mTargetPosition(0) -= default_angle * M_PI/180.0;
	if (mTargetPosition(0) < -M_PI) mTargetPosition(0) += 2*M_PI;
        break;
      case 'm':
        singlestep = !singlestep;
        break;
      case 'n':
        mController->update(mTargetPosition);
        mWorld->step();
        break;
      case 'v':
        mShow3D = !mShow3D;
        break;
      default:
        SimWindow::keyboard(key, x, y);
      }
  }

  void timeStepping() override
  {

    if (singlestep)
      return;

    //std::cout << "FC: " << mStick->getBodyNode("cylinder_link")->getFrictionCoeff() << std::endl;
    
    mController->update(mTargetPosition);

    // Show Forces
    {
      Eigen::VectorXd corforces = mStick->getConstraintForces();
      std::cout << "Constraint Forces: " << corforces << std::endl;
    }

    {
      Eigen::Quaterniond Rot = mTrackBall.getCurrQuat();
      std::cout << "Zoom: " << mZoom << "Trans: " << mTrans << std::endl;
      std::cout << "Rot" << Rot.w() << Rot.vec() << std::endl;
    }

    // Show Position in World Coordinates
    {
      //BodyNodePtr bn = mStick->getBodyNode("cylinder_link");
      //Eigen::Vector3d pos = bn->getWorldTransform().translation();
      //std::cout << "Pos: " << pos << std::endl;
      //Eigen::AngleAxisd aa(bn->getWorldTransform().linear());
      //std::cout << "aa: " << aa << std::endl;
      //std::cout << "Angle: " << aa.angle() << "Axis : " << aa.axis() << std::endl;
      //dart::dynamics::Joint *j = mStick->getJoint("cylinder_joint");
      //std::cout << "Pos: " << j->getPositions() << std::endl;
    }

    // Step the simulation forward
    SimWindow::timeStepping();
  }

protected:

  /// The stick
  SkeletonPtr mStick;

  // The ground floor with the hole
  SkeletonPtr mFloor;

  // Target position of the stick
  Eigen::Vector6d mTargetPosition;

  // If true, then single stepping is possible
  bool singlestep;

  // True: The default 3D Window visualization
  // False: The 2D Projection
  bool mShow3D = true;

  // The controller for the stick
  Controller* mController;

  // Camera Rotation is coded in Quaternion of Trackball.
  Eigen::Quaterniond CamRot;
};


int main(int argc, char* argv[])
{

  MyWindow *wp = new MyWindow;

  int myargc = 0;
  char *myargv = NULL;

  
  // Print instructions
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'v': switch between 2D and 3D view" << std::endl;
  std::cout << "'q': Increase target x position" << std::endl;
  std::cout << "'a': Decrease target x position" << std::endl;
  std::cout << "'w': Increase target y position" << std::endl;
  std::cout << "'s': Decrease target y position" << std::endl;
  std::cout << "'e': Increase target z position" << std::endl;
  std::cout << "'d': Decrease target z position" << std::endl;
  std::cout << "'r': Increase rotation around x axis" << std::endl;
  std::cout << "'f': Decrease rotation around x axis" << std::endl;

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&myargc, &myargv);
  wp->initWindow(640, 480, "Machine Learning Hole");
  glutSetCursor(GLUT_CURSOR_CROSSHAIR);

  //sleep(2);
  glutMainLoopEvent();

  //sleep(5);
  
  while(1)
    glutMainLoopEvent();
}

extern "C"
{

static bool glut_is_initialized = false;

DartSim *dart_new(){
  DartSim *ds = (DartSim *)malloc(sizeof(DartSim));
  ds->simtime = 0.0;

  MyWindow *wp;
  int argc = 0;
  char *argv = NULL;

  // Create a window for rendering the world and handling user input
  wp = new MyWindow;
  if (!glut_is_initialized){
    glutInit(&argc, &argv);
    glut_is_initialized = true;
  }
  wp->initWindow(640, 480, "Machine Learning Hole");
  if (!glut_is_initialized)
    glutSetCursor(GLUT_CURSOR_CROSSHAIR);
  
  ds->mysim = reinterpret_cast<mlhole*>(wp);

  return ds;
}

void dart_gc(DartSim *ds){
  if (ds){
    std::cout << "Destructor" << ds->simtime << std::endl;
    delete reinterpret_cast<MyWindow *>(ds->mysim);
    free(ds);
  }
}

double dart_act(DartSim *ds,
                int action){
  ds->simtime += (double) action;
  glutMainLoopEvent();
  return ds->simtime;
}

} // extern "C"
