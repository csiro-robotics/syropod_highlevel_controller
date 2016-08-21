#pragma once
#include "model.h"
#include "debugOutput.h"

//States for walking controller
enum RobotState
{
  STARTING,
  MOVING,
  STOPPING,  
  STOPPED
};

enum WalkState
{
  SWING,
  STANCE,
  SWING_TRANSITION,
  STANCE_TRANSITION,
  FORCE_STANCE,
  FORCE_STOP
};

//Controller that handles walking
struct WalkController
{
  Model *model;
  
  Pose pose; //DEBUGGING
  
  Parameters params;
  
  RobotState state = STOPPED;
  
  double timeDelta;
  
  //Walk parameters
  double stepFrequency;  
  double stepClearance;
  double bodyClearance;
  
  int phaseLength;
  int stanceEnd;   
  int swingStart;
  int swingEnd;     
  int stanceStart;
  
  std::vector<int> legSelectionPattern;
  std::vector<int> sideSelectionPattern;
  
  Vector3d footSpreadDistances;
  double minFootprintRadius;
  double stanceRadius; // affects turning circle
  Vector3d identityTipPositions[3][2];
  Vector2d localCentreVelocity;
  Vector2d localCentreAcceleration;
  double angularVelocity;
  double maximumBodyHeight;
  
  int targetsMet = 0;
    
  struct LegStepper
  {
    bool metTarget = false;
    
    int phase;
    int phaseOffset;
    
    WalkState state = STANCE;
    
    Vector2d strideVector; // length gives stride length
    Vector3d currentTipPosition;
    Vector3d originTipPosition;
    Vector3d defaultTipPosition;
    
    struct WalkController *walker; //So LegStepper can access walkcontroller member variables
    struct Parameters *params;
    
    void updateSwingPos(Vector3d *pos);
    void updatePosition();
    
  } legSteppers[3][2];
  
  // Determines the basic stance pose which the hexapod will try to maintain, by finding the largest footprint 
  // radius that each leg can achieve for the specified level of clearance
  //    stepFrequency- preferred step cycles per second
  //    bodyClearance, stepClearance- 0 to 1, 1 is vertical legs
  //    stanceLegYaws- natural yaw pose per leg
  //    minYawLimits- the minimum yaw (or hip) joint limit around centre for each leg

  WalkController(Model *model, Parameters p);
  
  void init(Model *model, Parameters p);
  // curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
  // bodyOffset is body pose relative to the basic stance pose, 
  // note that large offsets may prevent achievable leg positions
  // call this function even when not walking (newLocalVelocity=0), otherwise joint angles will just freeze
  void updateWalk(Vector2d newLocalVelocity, double newCurvature, double deltaZ[3][2], double velocityMultiplier = 1.0);
  void setGaitParams(Parameters p);
};