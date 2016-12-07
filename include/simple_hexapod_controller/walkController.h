#pragma once

#include "model.h"
#include "debugOutput.h"

//Controller that handles walking
struct WalkController
{
  Model *model;
  
  Pose pose; //DEBUGGING
  
  Parameters params;
  
  WalkState walkState = STOPPED;
  
  Vector3d tipPositions[3][2];
  
  double timeDelta;
  
  //Walk parameters
  double stepFrequency;  
  double stepClearance;
  double stepDepth;
  double bodyClearance;
  
  int phaseLength;
  int stanceEnd;   
  int swingStart;
  int swingEnd;     
  int stanceStart;
  
  Vector3d footSpreadDistances;
  double minFootprintRadius;
  double stanceRadius; // affects turning circle
  Vector3d identityTipPositions[3][2];
  
  Vector2d currentLinearVelocity; //Linear Body Velocity
  double currentAngularVelocity; //Angular Body Velocity
  
  double maximumBodyHeight;
  
  int legsInCorrectPhase = 0;
  int legsCompletedFirstStep = 0;
  
  StepState stepStates[3][2];
    
  struct LegStepper
  {
    int legIndex;
    int sideIndex;
    
    bool inCorrectPhase = false;
    bool completedFirstStep = false;
    
    int phase;
    int phaseOffset;
    
    double swingProgress;
    double stanceProgress;
    
    StepState stepState = STANCE;
    
    Vector3d swing1ControlNodes[5];	//Primary swing bezier curve
    Vector3d swing2ControlNodes[5];	//Secondary swing bezier curve
    Vector3d stanceControlNodes[5];	//Stance bezier curve
    
    double swingDeltaT;
    double stanceDeltaT;
    
    Vector2d strideVector; // length gives stride length
    Vector3d currentTipPosition;
    Vector3d currentTipVelocity;
    Vector3d defaultTipPosition;
    Vector3d swingOriginTipPosition;
    Vector3d stanceOriginTipPosition;

    
    struct WalkController *walker; //So LegStepper can access walkcontroller member variables
    struct Parameters *params;
    
    void updatePosition();

    void generateSwingControlNodes(Vector3d strideVector);
    void generateStanceControlNodes(Vector3d strideVector);
    
    void iteratePhase();
    
    void setState(StepState state);
    
    double calculateDeltaT(StepState state, int length);
    
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
  void updateWalk(Vector2d linearVelocityInput, double angularVelocityInput);
  void updateManual(LegDesignation primaryLegSelection, Vector3d primaryTipVelocityInput, 
		    LegDesignation secondaryLegSelection, Vector3d secondaryTipVelocityInput);
  void setGaitParams(Parameters p);
};