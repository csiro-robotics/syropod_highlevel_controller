#pragma once
#include "../include/simple_hexapod_controller/tripodWalk.h"

#define STANCE_PHASE (5*pi) //WAVE: 5*pi	TRIPOD: pi	RIPPLE: 2*pi	
#define SWING_PHASE (pi)    //WAVE: pi	TRIPOD: pi	RIPPLE: pi
#define PHASE_OFFSET (pi)
#define STANCE_FUNC_ORDER (8*STANCE_PHASE)/SWING_PHASE
#define HEIGHT_RATIO 0.25 //The ratio between the positive and negative lift heights (stance/swing)

#define SWING_START (STANCE_PHASE/2)
#define SWING_END ((STANCE_PHASE/2)+SWING_PHASE)

#define LEG_SELECTION_PATTERN {0,1,2,0,1,2}		//WAVE: {0,1,2,0,1,2}	TRIPOD: {0,1,2,0,1,2}	RIPPLE: {2,1,0,2,1,0}
#define SIDE_SELECTION_PATTERN {0,0,0,1,1,1}	//WAVE: {0,0,0,1,1,1}	TRIPOD: {0,0,0,1,1,1}	RIPPLE: {1,0,1,0,1,0}

#define MAX_POS 1000.0

#define MAX_ACCELERATION 0.1
#define MAX_CURVATURE_SPEED 0.4

#define FOOTPRINT_DOWNSCALE 0.8			// this is because the step cycle exceeds the ground footprint in order to maintain velocity
#define STEP_CURVATURE_ALLOWANCE 0.7 	// dont need full height cylinder (when 1) because the top of the step is rounded

/***********************************************************************************************************************
***********************************************************************************************************************/

Vector3d GaitController::LegStepper::getPosition(double liftHeight)
{
  Vector3d strideVec(strideVector[0], strideVector[1], 0);
  if (phase > SWING_START && phase < SWING_END)
  {
    Vector3d nodes[4];
    nodes[0] = -strideVec*0.5;
    nodes[3] = strideVec*0.5;
    Vector3d lift(0,0,liftHeight*4.0);
    nodes[1] = nodes[0] + (lift - strideVec)/3.0;
    nodes[2] = nodes[3] + (lift + strideVec)/3.0;
    Vector3d pos = cubicBezier(nodes, (phase-STANCE_PHASE/2)/SWING_PHASE);
    ASSERT(pos.squaredNorm() < MAX_POS);
    return pos;
  }
  else // stance
  {
    double t = 2*phase/STANCE_PHASE;
    if (t > 1.0)
      t -= 2*(STANCE_PHASE+SWING_PHASE)/STANCE_PHASE;
    Vector3d pos = strideVec * 0.5 * -t;
    pos[2] = liftHeight*HEIGHT_RATIO* (pow(t, STANCE_FUNC_ORDER) - 1.0);
    ASSERT(pos.squaredNorm() < MAX_POS);
    return pos;
  }
}
/***********************************************************************************************************************
 * Determines the basic stance pose which the hexapod will try to maintain, by 
 * finding the largest footprint radius that each leg can achieve for the 
 * specified level of clearance.
 * 
 * stepFrequency:	preferred step cycles per second
 * stepClearance:	1 is full leg length, smaller values allow the leg more lateral movement 
 * 					(which is stored as minFootprintRadius)
 * stanceLegYaws:	natural yaw pose per leg
 * minYawLimits:	the minimum yaw (or hip) joint limit around centre for each leg
 * maximumKneeBend:	in radians, 0 can't bend knee at all
 * bodyClearance:	0 to 1, 1 is vertical legs. Default calculates best clearance for given leg clearance
 * 
***********************************************************************************************************************/

GaitController::GaitController( Model *model, 
                                int gaitType, 
                                double stepFrequency, 
                                double stepClearance, 
                                const Vector3d &stanceLegYaws, 
                                const Vector3d &yawLimitAroundStance, 
                                double maximumKneeBend, 
                                double bodyClearance) : 
    model(model),
    gaitType(gaitType),
    stepFrequency(stepFrequency), 
    bodyClearance(bodyClearance), 
    stepClearance(stepClearance), 
    stanceLegYaws(stanceLegYaws), 
    walkPhase(0)
{
    ASSERT(stepClearance >= 0 && stepClearance < 1.0);
    ASSERT(stepClearance <= 2.0*model->legs[0][0].femurLength / model->legs[0][0].legLength); // Can't to lift this high

    // If undefined - work out a best value to maximise circular footprint for given step clearance
    if (bodyClearance == -1) 
    {
        // in this case we assume legs have equal characteristics
        bodyClearance = model->legs[0][0].getMinLength(maximumKneeBend)/model->legs[0][0].legLength +
                        STEP_CURVATURE_ALLOWANCE*stepClearance;
                        
        ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);
    }

    minFootprintRadius = 1e10;

    for (int l = 0; l<3; l++)
    {
        // find biggest circle footprint inside the pie segment defined by the body clearance and the yaw limits
        Leg &leg = model->legs[l][0];
        double horizontalRange = sqrt(sqr(leg.legLength) - sqr(bodyClearance*leg.legLength));
        double theta = yawLimitAroundStance[l];
        double cotanTheta = tan(0.5*pi - theta);
        double rad = solveQuadratic(sqr(cotanTheta), 2.0*horizontalRange, -sqr(horizontalRange));

        // we should also take into account the stepClearance not getting too high for the leg to reach
        double minLegLength = leg.getMinLength(maximumKneeBend);
        double legTipBodyClearance = max(0.0, bodyClearance - STEP_CURVATURE_ALLOWANCE*stepClearance)*leg.legLength; 
        
        if (legTipBodyClearance < minLegLength)
        {
            double liftRad = (horizontalRange - sqrt(sqr(minLegLength) - sqr(legTipBodyClearance))) / 2.0;
            // footprint radius due to lift is smaller than due to yaw limits, so reduce this minimum radius
            if (liftRad < rad)
            {
                rad = liftRad;
            }
        }

        footSpreadDistances[l] = horizontalRange - rad;
        minFootprintRadius = min(minFootprintRadius, rad*FOOTPRINT_DOWNSCALE);

        for (int side = 0; side<2; side++)
        {
            localStanceTipPositions[l][side] = 	model->legs[l][side].rootOffset + 
                footSpreadDistances[l]*Vector3d(cos(stanceLegYaws[l]), sin(stanceLegYaws[l]), 0) + 
                Vector3d(0,0,-bodyClearance*model->legs[l][side].legLength);
            localStanceTipPositions[l][side][0] *= model->legs[l][side].mirrorDir;
        }
    }

    stanceRadius = abs(localStanceTipPositions[1][0][0]);

    for (int i = 0; i<6; i++)
    {	  
        int l[6] = LEG_SELECTION_PATTERN;
        int s[6] = SIDE_SELECTION_PATTERN;
        legSteppers[l[i]][s[i]].phaseOffset = PHASE_OFFSET*double(i);
        legSteppers[l[i]][s[i]].phase = 0; // Ensures that feet start stepping naturally and don't pop to up position
        legSteppers[l[i]][s[i]].strideVector = Vector2d(0,0);
    }

    localCentreVelocity = Vector2d(0,0);
    angularVelocity = 0;
    localCentreVelocity = Vector2d(0,0);

    pose.rotation = Quat(1,0,0,0);
    pose.position = Vector3d(0, 0, bodyClearance*model->legs[0][0].legLength);
}

/***********************************************************************************************************************
 * curvature:	0 to 1 - where 1 is rotate on the spot and 0.5 rotates around leg stance pos
 * bodyOffset:	Body-pose relative to the basic stance pose - Note: large offsets may prevent achievable leg positions

***********************************************************************************************************************/

void GaitController::update(Vector2d localNormalisedVelocity, double newCurvature, const Pose *bodyOffset)
{
    targets.clear();

    Vector2d localVelocity = localNormalisedVelocity*minFootprintRadius*2.0*stepFrequency;
    double normalSpeed = localVelocity.norm();
    ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this

    // this block assures the local velocity and curvature values don't change too quickly
    bool isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;
    if (normalSpeed > 0.0 && !isMoving) // started walking again
    {
        // reset, and we want to pick the walkPhase closest to its current phase or antiphase...
        if (walkPhase > STANCE_PHASE + SWING_PHASE/2)
        {
            walkPhase = STANCE_PHASE+SWING_PHASE;
        }
        else
        {
            walkPhase = STANCE_PHASE/2;
        }
    }

    // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
    double newAngularVelocity = newCurvature * normalSpeed/stanceRadius;
    double dif = newAngularVelocity - angularVelocity;

    if (abs(dif)>0.0)
    {
        angularVelocity += dif * min(1.0, MAX_CURVATURE_SPEED*timeDelta/abs(dif));
    }

    Vector2d centralVelocity = localVelocity * (1 - abs(newCurvature));
    Vector2d diff = centralVelocity - localCentreVelocity;
    double diffLength = diff.norm();

    if (diffLength > 0.0)
    {
        localCentreVelocity += diff * min(1.0, MAX_ACCELERATION*timeDelta/diffLength);
    }

    //Iterate master walk phase
    double phaseLength = STANCE_PHASE + SWING_PHASE;
    walkPhase += phaseLength*stepFrequency*timeDelta;
    if (walkPhase > phaseLength)
    {
        walkPhase -= phaseLength;
    }

    isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;

    for (int l = 0; l<3; l++)
    {
        for (int s = 0; s<2; s++)
        {
            LegStepper &legStepper = legSteppers[l][s];
            Leg &leg = model->legs[l][s];
            
            legStepper.strideVector = (STANCE_PHASE/(STANCE_PHASE+SWING_PHASE))*
                (localCentreVelocity + angularVelocity*Vector2d(leg.localTipPosition[1], -leg.localTipPosition[0]))/
                stepFrequency;
            
            double phase = fmod(walkPhase + legStepper.phaseOffset, phaseLength);
            
            // if stopped then do nothing until phase gets reset to 0 (new step)
            if (legStepper.phase == STANCE_PHASE+SWING_PHASE && phase >= (STANCE_PHASE+SWING_PHASE)/2 && normalSpeed>0) 
            {
                localCentreVelocity = Vector2d(1e-10, 1e-10);
                angularVelocity = 1e-10;
            }
            // if just stopped, don't continue leg cycle beyond 2pi
            else if (phase < legStepper.phase && !isMoving) 
            {
                legStepper.phase = STANCE_PHASE+SWING_PHASE;
            }
            else
            {
                legStepper.phase = phase; // otherwise follow the step cycle exactly
            }
            
            Vector3d pos = localStanceTipPositions[l][s] + legStepper.getPosition(stepClearance*leg.legLength);
            
            if ((legStepper.phase < SWING_START) || (legStepper.phase > SWING_END))
            {
                targets.push_back(pose.transformVector(pos));
            }
            
            leg.applyLocalIK(pos);
        }
    }

    if (bodyOffset != NULL)
    {
        for (int l = 0; l<3; l++)
        {
            for (int s = 0; s<2; s++)
            {
                model->legs[l][s].applyLocalIK(bodyOffset->inverseTransformVector(model->legs[l][s].localTipPosition), false); 
                    // false means we don't update local tip position, as it is needed above in step calculations
            }
        }
    }

    Vector2d push = localCentreVelocity*timeDelta;
    pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
    pose.rotation *= Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta));  
}
