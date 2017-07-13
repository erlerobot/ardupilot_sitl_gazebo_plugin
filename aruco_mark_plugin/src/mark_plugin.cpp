#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// Circle radius
#define RADIUS 1

// Square height and width
#define HEIGHT 0.5
#define WIDTH  1

// Define shapes
#define CIRCLE 0
#define SQUARE 1
#define STOP   2
#define DOT    2 // DOT == STOP

//#define STEP   0.001

// Square status
#define INITIAL 0
#define SIDE0   1
#define SIDE1   2
#define SIDE2   3
#define SIDE3   4

namespace gazebo
{
  class MarkPlugin : public ModelPlugin
  {

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //private: math::Pose initialPose;
    private: double initialX, initialY, initialZ;
    private: double initialRoll, initialPitch, initialYaw;
    private: float angle;
    private: int shape;

    private: float X, Y;

    private: float STEP;

    private: int status;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      angle = 0;

      math::Pose initialPose = this->model->GetWorldPose();
      initialX      = initialPose.pos[0];
      initialY      = initialPose.pos[1];
      initialZ      = initialPose.pos[2];
      initialRoll   = initialPose.pos[3];
      initialPitch  = initialPose.pos[4];
      initialYaw    = initialPose.pos[5];

      std::string shapeParam = _sdf->Get<std::string>("shape");
      std::transform(shapeParam.begin(), shapeParam.end(), shapeParam.begin(), ::tolower);

      if(shapeParam == "square"){
        shape = SQUARE;
      } else if ( shapeParam == "circle"){
        shape = CIRCLE;
      } else if ( shapeParam == "dot"){
        shape = DOT;
      } else if ( shapeParam == "stop"){
        shape = STOP;
      } else {
        shape = STOP;
      }

      // Hardcoded for testing
      // shape = SQUARE; 
      STEP = 0.0002; 

      status = INITIAL;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MarkPlugin::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Draw a circle
      //this->model->SetLinearVel(math::Vector3(1, 0, 0));
      //this->model->SetAngularVel(math::Vector3(0, 0, 1));

      switch (shape)
      {
        case CIRCLE:
        {
          angle = angle + 0.0004;
          X = initialX + sin(angle) * RADIUS;
          Y = initialY + cos(angle) * RADIUS;
          math::Pose pose = math::Pose(math::Vector3(X,Y,initialZ), math::Quaternion(math::Vector3(initialRoll,initialPitch,initialYaw)));
          this->model->SetWorldPose(pose);
          break;
        } // end case CIRCLE
          

        case SQUARE:
        {
          /*
            Draw a square in this order:

            (1)------->(2)
             ^          |
             |          v
            (0)<-------(3)

            (0) : (initialX, initialY)
            (1) : (initialX, initialY + HEIGHT)
            (2) : (initialX + WIDTH, initialY + HEIGHT)
            (3) : (initialX + WIDTH, initialY)
          */

          switch (status)
          {

            case (INITIAL):
            {
              X = (float)initialX;
              Y = (float)initialY;
              status = SIDE0;
              break;
            }

            case (SIDE0):
            {
              /*
                (1)
                 ^
                 |
                (0)
              */
              Y = Y + STEP;
              math::Pose pose = math::Pose(math::Vector3(X,Y,initialZ), math::Quaternion(math::Vector3(initialRoll,initialPitch,initialYaw)));
              this->model->SetWorldPose(pose);
              if (Y >= (initialY + HEIGHT)){
                status = SIDE1;
              }
              break;
            }

            case (SIDE1):
            {
              /*
                (1)------->(2)
              */
              X = X + STEP;
              math::Pose pose = math::Pose(math::Vector3(X,Y,initialZ), math::Quaternion(math::Vector3(initialRoll,initialPitch,initialYaw)));
              this->model->SetWorldPose(pose);
              if (X >= (initialX + WIDTH)){
                status = SIDE2;
              }
              break;
            }

            case (SIDE2):
            {
              /*
                (2)
                 |
                 v
                (3)
              */
              Y = Y - STEP;
              math::Pose pose = math::Pose(math::Vector3(X,Y,initialZ), math::Quaternion(math::Vector3(initialRoll,initialPitch,initialYaw)));
              this->model->SetWorldPose(pose);
              if (Y <= initialY){
                status = SIDE3;
              }
              break;
            }

            case (SIDE3):
            {
              /*
                (0)<-------(3)
              */
              X = X - STEP;
              math::Pose pose = math::Pose(math::Vector3(X,Y,initialZ), math::Quaternion(math::Vector3(initialRoll,initialPitch,initialYaw)));
              this->model->SetWorldPose(pose);

              if(X <= initialX){
                status = INITIAL;
              }
              break;
            }

          }  

          break;
          
        } // end case SQUARE

      case (STOP):
      {
        break;
      }  // end case STOP

      break;

      } // end switch shape

    } // end void OnUpdate

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MarkPlugin)
} // end namespace GAZEBO
