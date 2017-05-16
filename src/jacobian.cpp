/*******************************************************************************************************************//**
 *  \file    jacobian.cpp
 *  \brief   Defines the generation of jacobians for various degrees of freedom.
 *
 *  \author Fletcher Talbot
 *  \date   June 2017
 *  \version 0.5.0
 *
 *  CSIRO Autonomous Systems Laboratory
 *  Queensland Centre for Advanced Technologies
 *  PO Box 883, Kenmore, QLD 4069, Australia
 *
 *  (c) Copyright CSIRO 2017
 *
 *  All rights reserved, no part of this program may be used
 *  without explicit permission of CSIRO
 *
***********************************************************************************************************************/

#include "../include/simple_hexapod_controller/standard_includes.h"

/***********************************************************************************************************************
 * Jacobian functions from DH parameters (derived analytically) (en.wikipedia.org/wiki/Denavit–Hartenberg_parameters)
***********************************************************************************************************************/
MatrixXd createJacobian1DOF(vector<map<string, double>> dh)
{
  //double d1 = dh[0]["d"]; //Unused
  double r1 = dh[0]["r"];
  double sT1 = sin(dh[0]["theta"]);
  double cT1 = cos(dh[0]["theta"]);
  //double sA1 = sin(dh[0]["alpha"]); //Unused
  //double cA1 = cos(dh[0]["alpha"]); //Unused
  MatrixXd j(3, 1);
  j(0,0) = -r1*sT1;
  j(1,0) = r1*cT1;
  j(2,0) = 0.0;
  return j;
}
      
MatrixXd createJacobian2DOF(vector<map<string, double>> dh)
{
  //double d1 = dh[0]["d"]; //Unused
  double d2 = dh[1]["d"];
  double r1 = dh[0]["r"];
  double r2 = dh[1]["r"];
  double sT1 = sin(dh[0]["theta"]);
  double sT2 = sin(dh[1]["theta"]);
  double cT1 = cos(dh[0]["theta"]);
  double cT2 = cos(dh[1]["theta"]);
  double sA1 = sin(dh[0]["alpha"]);
  //double sA2 = sin(dh[1]["alpha"]); //Unused
  double cA1 = cos(dh[0]["alpha"]);
  //double cA2 = cos(dh[1]["alpha"]); //Unused
  MatrixXd j(3, 2);
  j(0,0) = -(sT1*r2*cT2)
            -(cT1*cA1*r2*sT2)
            +(cT1*sA1*d2)
            -(r1*sT1);
  j(0,1) = -(cT1*r2*sT2)
            -(sT1*cA1*r2*cT2);
  j(1,0) = (cT1*r2*cT2)
            -(sT1*cA1*r2*sT2)
            +(sT1*sA1*d2)+(r1*cT1);
  j(1,1) = -(sT1*r2*sT2)
            +(cT1*cA1*r2*cT2);
  j(2,0) = 0.0;
  j(2,1) = (sA1*r2*cT2);
  return j;
}

MatrixXd createJacobian3DOF(vector<map<string, double>> dh)
{
  //double d1 = dh[0]["d"]; //Unused
  double d2 = dh[1]["d"];
  double d3 = dh[2]["d"];
  double r1 = dh[0]["r"];
  double r2 = dh[1]["r"];
  double r3 = dh[2]["r"];
  double sT1 = sin(dh[0]["theta"]);
  double sT2 = sin(dh[1]["theta"]);
  double sT3 = sin(dh[2]["theta"]);
  double cT1 = cos(dh[0]["theta"]);
  double cT2 = cos(dh[1]["theta"]);
  double cT3 = cos(dh[2]["theta"]);
  double sA1 = sin(dh[0]["alpha"]);
  double sA2 = sin(dh[1]["alpha"]);
  //double sA3 = sin(dh[2]["alpha"]); //Unused
  double cA1 = cos(dh[0]["alpha"]);
  double cA2 = cos(dh[1]["alpha"]);
  //double cA3 = cos(dh[2]["alpha"]); //Unused
  MatrixXd j(3,3);
  j(0,0) = -(sT1*cT2*r3*cT3)
            -(cT1*cA1*sT2*r3*cT3)
            +(sT1*sT2*cA2*r3*sT3)
            -(cT1*cA1*cT2*cA2*r3*sT3)
            +(cT1*sA1*sA2*r3*sT3)
            -(sT1*sT2*sA2*d3)
            +(cT1*cA1*cT2*sA2*d3)
            +(cT1*sA1*cA2*d3)
            -(sT1*r2*cT2)
            -(cT1*cA1*r2*sT2)
            +(cT1*sA1*d2)-(r1*sT1);
  j(0,1) = -(cT1*sT2*r3*cT3)
            -(sT1*cA1*cT2*r3*cT3)
            -(cT1*cT2*cA2*r3*sT3)
            +(sT1*cA1*sT2*cA2*r3*sT3)
            +(cT1*cT2*sA2*d3)
            -(sT1*cA1*sT2*sA2*d3)
            -(cT1*r2*sT2)
            -(sT1*cA1*r2*cT2);
  j(0,2) = -(cT1*cT2*r3*sT3)
            +(sT1*cA1*sT2*r3*sT3)
            -(cT1*sT2*cA2*r3*cT3)
            -(sT1*cA1*cT2*cA2*r3*cT3)
            +(sT1*sA1*sA2*r3*cT3);
  j(1,0) = (cT1*cT2*r3*cT3)
            -(sT1*cA1*sT2*r3*cT3)
            -(cT1*sT2*cA2*r3*sT3)
            -(sT1*cA1*cT2*cA2*r3*sT3)
            +(sT1*sA1*sA2*r3*sT3)
            +(cT1*sT2*sA2*d3)
            +(sT1*cA1*cT2*sA2*d3)
            +(sT1*sA1*cA2*d3)
            +(cT1*r2*cT2)
            -(sT1*cA1*r2*sT2)
            +(sT1*sA1*d2)
            +(r1*cT1);
  j(1,1) = -(sT1*sT2*r3*cT3)
            +(cT1*cA1*cT2*r3*cT3)
            -(sT1*cT2*cA2*r3*sT3)
            -(cT1*cA1*sT2*cA2*r3*sT3)
            +(sT1*cT2*sA2*d3)
            +(cT1*cA1*sT2*sA2*d3)
            -(sT1*r2*sT2)
            +(cT1*cA1*r2*cT2);
  j(1,2) = -(sT1*cT2*r3*sT3)
            -(cT1*cA1*sT2*r3*sT3)
            -(sT1*sT2*cA2*r3*cT3)
            +(cT1*cA1*cT2*cA2*r3*cT3)
            -(cT1*sA1*sA2*r3*cT3);
  j(2,0) = 0;
  j(2,1) = (sA1*cT2*r3*cT3)
            -(sA1*sT2*cA2*r3*sT3)
            +(sA1*sT2*sA2*d3)
            +(sA1*r2*cT2);
  j(2,2) = -(sA1*sT2*r3*sT3)
            +(sA1*cT2*cA2*r3*cT3)
            +(cA1*sA2*r3*cT3);
  return j;
}

MatrixXd createJacobian4DOF(vector<map<string, double>> dh)
{
  //double d1 = dh[0]["d"]; //Unused
  double d2 = dh[1]["d"];
  double d3 = dh[2]["d"];
  double d4 = dh[3]["d"];
  double r1 = dh[0]["r"];
  double r2 = dh[1]["r"];
  double r3 = dh[2]["r"];
  double r4 = dh[3]["r"];
  double sT1 = sin(dh[0]["theta"]);
  double sT2 = sin(dh[1]["theta"]);
  double sT3 = sin(dh[2]["theta"]);
  double sT4 = sin(dh[3]["theta"]);
  double cT1 = cos(dh[0]["theta"]);
  double cT2 = cos(dh[1]["theta"]);
  double cT3 = cos(dh[2]["theta"]);
  double cT4 = cos(dh[3]["theta"]);
  double sA1 = sin(dh[0]["alpha"]);
  double sA2 = sin(dh[1]["alpha"]);
  double sA3 = sin(dh[2]["alpha"]);
  //double sA4 = sin(dh[3]["alpha"]); //Unused
  double cA1 = cos(dh[0]["alpha"]);
  double cA2 = cos(dh[1]["alpha"]);
  double cA3 = cos(dh[2]["alpha"]);
  //double cA4 = cos(dh[3]["alpha"]); //Unused
  MatrixXd j(3,4);
  j(0,0) = -(sT1*cT2*cT3*r4*cT4)
            -(cT1*cA1*sT2*cT3*r4*cT4)
            +(sT1*sT2*cA2*sT3*r4*cT4)
            -(cT1*cA1*cT2*cA2*sT3*r4*cT4)
            -(cT1*sA1*sA2*sT3*r4*cT4)
            +(sT1*cT2*sT3*cA3*r4*sT4)
            +(cT1*cA1*sT2*sT3*cA3*r4*sT4)
            +(sT1*sT2*cA2*cT3*cA3*r4*sT4)
            -(cT1*cA1*cT2*cA2*cT3*cA3*r4*sT4)
            +(cT1*sA1*sA2*cT3*cA3*r4*sT4)
            -(sT1*sT2*sA2*sA3*r4*sT4)
            +(cT1*cA1*cT2*sA2*sA3*r4*sT4)
            +(cT1*sA1*cA2*sA3*r4*sT4)
            -(sT1*cT2*sT3*sA3*d4)
            -(cT1*cA1*sT2*sT3*sA3*d4)
            -(sT1*sT2*cA2*cT3*sA3*d4)
            +(cT1*cA1*cT2*cA2*cT3*sA3*d4)
            -(cT1*sA1*sA2*cT3*sA3*d4)
            -(sT1*sT2*sA2*cA3*d4)
            +(cT1*cA1*cT2*sA2*cA3*d4)
            -(sT1*cT2*r3*cT3)
            +(cT1*sA1*cA2*cA3*d4)
            -(cT1*cA1*sT2*r3*cT3)
            +(sT1*sT2*cA2*r3*sT3)
            -(cT1*cA1*cT2*cA2*r3*sT3)
            +(cT1*sA1*sA2*r3*sT3)
            -(sT1*sT2*sA2*d3)
            +(cT1*cA1*cT2*sA2*d3)
            +(cT1*sA1*cA2*d3)
            -(sT1*r2*cT2)
            -(cT1*cA1*r2*sT2)
            +(cT1*sA1*d2)
            -(r1*sT1);
  j(0,1) = -(cT1*sT2*cT3*r4*cT4)
            -(sT1*cA1*cT2*cT3*r4*cT4)
            -(cT1*cT2*cA2*sT3*r4*cT4)
            +(sT1*cA1*sT2*cA2*sT3*r4*cT4)
            +(cT1*sT2*sT3*cA3*r4*sT4)
            +(sT1*cA1*cT2*sT3*cA3*r4*sT4)
            -(cT1*cT2*cA2*cT3*cA3*r4*sT4)
            +(sT1*cA1*sT2*cA2*cT3*cA3*r4*sT4)
            +(cT1*cT2*sA2*sA3*r4*sT4)
            -(sT1*cA1*sT2*sA2*sA3*r4*sT4)
            -(cT1*sT2*sT3*sA3*d4)
            -(sT1*cA1*cT2*sT3*sA3*d4)
            +(cT1*cT2*cA2*cT3*sA3*d4)
            -(sT1*cA1*sT2*cA2*cT3*sA3*d4)
            +(cT1*cT2*sA2*cA3*d4)
            -(sT1*cA1*sT2*sA2*cA3*d4)
            -(cT1*sT2*r3*cT3)
            -(sT1*cA1*cT2*r3*cT3)
            -(cT1*cT2*cA2*r3*sT3)
            +(sT1*cA1*sT2*cA2*r3*sT3)
            +(cT1*cT2*sA2*d3)
            -(sT1*cA1*sT2*sA2*d3)
            -(cT1*r2*sT2)
            -(sT1*cA1*r2*cT2);
  j(0,2) = -(cT1*cT2*sT3*r4*cT4)
            +(sT1*cA1*sT2*sT3*r4*cT4)
            -(cT1*sT2*cA2*cT3*r4*cT4)
            -(sT1*cA1*cT2*cA2*cT3*r4*cT4)
            +(sT1*sA1*sA2*cT3*r4*cT4)
            -(cT1*cT2*cT3*cA3*r4*sT4)
            +(sT1*cA1*sT2*cT3*cA3*r4*sT4)
            +(cT1*sT2*cA2*sT3*cA3*r4*sT4)
            +(sT1*cA1*cT2*cA2*sT3*cA3*r4*sT4)
            -(sT1*sA1*sA2*sT3*cA3*r4*sT4)
            +(cT1*cT2*cT3*sA3*d4)
            -(sT1*cA1*sT2*cT3*sA3*d4)
            -(cT1*sT2*cA2*sT3*sA3*d4)
            -(sT1*cA1*cT2*cA2*sT3*sA3*d4)
            +(sT1*sA1*sA2*sT3*sA3*d4)
            -(cT1*cT2*r3*sT3)
            +(sT1*cA1*sT2*r3*sT3)
            -(cT1*sT2*cA2*r3*cT3)
            -(sT1*cA1*cT2*cA2*r3*cT3)
            +(sT1*sA1*sA2*r3*cT3);
  j(0,3) = -(cT1*cT2*cT3*r4*sT4)
            +(sT1*cA1*sT2*cT3*r4*sT4)
            +(cT1*sT2*cA2*sT3*r4*sT4)
            +(sT1*cA1*cT2*cA2*sT3*r4*sT4)
            -(sT1*sA1*sA2*sT3*r4*sT4)
            -(cT1*cT2*sT3*cA3*r4*cT4)
            +(sT1*cA1*sT2*sT3*cA3*r4*cT4)
            -(cT1*sT2*cA2*cT3*cA3*r4*cT4)
            -(sT1*cA1*cT2*cA2*cT3*cA3*r4*cT4)
            +(sT1*sA1*sA2*cT3*cA3*r4*cT4)
            +(cT1*sT2*sA2*sA3*r4*cT4)
            +(sT1*cA1*cT2*sA2*sA3*r4*cT4)
            +(sT1*sA1*cA2*sA3*r4*cT4);
  j(1,0) = (cT1*cT2*cT3*r4*cT4)
            -(sT1*cA1*sT2*cT3*r4*cT4)
            -(cT1*sT2*cA2*sT3*r4*cT4)
            -(sT1*cA1*cT2*cA2*sT3*r4*cT4)
            +(sT1*sA1*sA2*sT3*r4*cT4)
            -(cT1*cT2*sT3*cA3*r4*sT4)
            +(sT1*cA1*sT2*sT3*cA3*r4*sT4)
            -(cT1*sT2*cA2*cT3*cA3*r4*sT4)
            -(sT1*cA1*cT2*cA2*cT3*cA3*r4*sT4)
            +(sT1*sA1*sA2*cT3*cA3*r4*sT4)
            +(cT1*sT2*sA2*sA3*r4*sT4)
            +(sT1*cA1*cT2*sA2*sA3*r4*sT4)
            +(sT1*sA1*cA2*sA3*r4*sT4)
            +(cT1*cT2*sT3*sA3*d4)
            -(sT1*cA1*sT2*sT3*sA3*d4)
            +(cT1*sT2*cA2*cT3*sA3*d4)
            +(sT1*cA1*cT2*cA2*cT3*sA3*d4)
            -(sT1*sA1*sA2*cT3*sA3*d4)
            +(cT1*sT2*sA2*cA3*d4)
            +(sT1*cA1*cT2*sA2*cA3*d4)
            +(sT1*sA1*cA2*cA3*d4)
            +(cT1*cT2*r3*cT3)
            -(sT1*cA1*sT2*r3*cT3)
            -(cT1*sT2*cA2*r3*sT3)
            -(sT1*cA1*cT2*cA2*r3*sT3)
            +(sT1*sA1*sA2*r3*sT3)
            +(cT1*sT2*sA2*d3)
            +(sT1*cA1*cT2*sA2*d3)
            +(sT1*sA1*cA2*d3)
            +(cT1*r2*cT2)
            -(sT1*cA1*r2*sT2)
            +(sT1*sA1*d2)
            +(r1*cT1);
  j(1,1) = -(sT1*sT2*cT3*r4*cT4)
            +(cT1*cA1*cT2*cT3*r4*cT4)
            -(sT1*cT2*cA2*sT3*r4*cT4)
            -(cT1*cA1*sT2*cA2*sT3*r4*cT4)
            +(sT1*sT2*sT3*cA3*r4*sT4)
            -(cT1*cA1*cT2*sT3*cA3*r4*sT4)
            -(sT1*cT2*cA2*cT3*cA3*r4*sT4)
            -(cT1*cA1*sT2*cA2*cT3*cA3*r4*sT4)
            +(sT1*cT2*sA2*sA3*r4*sT4)
            +(cT1*cA1*sT2*sA2*sA3*r4*sT4)
            -(sT1*sT2*sT3*sA3*d4)
            +(sT1*cT2*cA2*cT3*sA3*d4)
            +(cT1*cA1*sT2*cA2*cT3*sA3*d4)
            +(sT1*cT2*sA2*cA3*d4)
            +(cT1*cA1*sT2*sA2*cA3*d4)
            -(sT1*sT2*r3*cT3)
            +(cT1*cA1*cT2*r3*cT3)
            -(sT1*cT2*cA2*r3*sT3)
            -(cT1*cA1*sT2*cA2*r3*sT3)
            +(sT1*cT2*sA2*d3)
            +(cT1*cA1*sT2*sA2*d3)
            -(sT1*r2*sT2)
            +(cT1*cA1*r2*cT2);
  j(1,2) = -(sT1*cT2*sT3*r4*cT4)
            -(cT1*cA1*sT2*sT3*r4*cT4)
            -(sT1*sT2*cA2*cT3*r4*cT4)
            +(cT1*cA1*cT2*cA2*cT3*r4*cT4)
            -(cT1*sA1*sA2*cT3*r4*cT4)
            -(sT1*cT2*cT3*cA3*r4*sT4)
            -(cT1*cA1*sT2*cT3*cA3*r4*sT4)
            +(sT1*sT2*cA2*sT3*cA3*r4*sT4)
            -(cT1*cA1*cT2*cA2*sT3*cA3*r4*sT4)
            +(cT1*sA1*sA2*sT3*cA3*r4*sT4)
            +(sT1*cT2*cT3*sA3*d4)
            +(cT1*cA1*sT2*cT3*sA3*d4)
            -(sT1*sT2*cA2*sT3*sA3*d4)
            +(cT1*cA1*cT2*cA2*sT3*sA3*d4)
            -(cT1*sA1*sA2*sT3*sA3*d4)
            -(sT1*cT2*r3*sT3)
            -(cT1*cA1*sT2*r3*sT3)
            -(sT1*sT2*cA2*r3*cT3)
            +(cT1*cA1*cT2*cA2*r3*cT3)
            -(cT1*sA1*sA2*r3*cT3);
  j(1,3) = -(sT1*cT2*cT3*r4*sT4)
            -(cT1*cA1*sT2*cT3*r4*sT4)
            +(sT1*sT2*cA2*sT3*r4*sT4)
            -(cT1*cA1*cT2*cA2*sT3*r4*sT4)
            +(cT1*sA1*sA2*sT3*r4*sT4)
            -(sT1*cT2*sT3*cA3*r4*cT4)
            -(cT1*cA1*sT2*sT3*cA3*r4*cT4)
            -(sT1*sT2*cA2*cT3*cA3*r4*cT4)
            +(cT1*cA1*cT2*cA2*cT3*cA3*r4*cT4)
            -(cT1*sA1*sA2*cT3*cA3*r4*cT4)
            +(sT1*sT2*sA2*sA3*r4*cT4)
            -(cT1*cA1*cT2*sA2*sA3*r4*cT4)
            -(cT1*sA1*cA2*sA3*r4*cT4);
  j(2,0) = 0;
  j(2,1) = (sA1*cT2*cT3*r4*cT4)
            -(sA1*sT2*cA2*sT3*r4*cT4)
            -(sA1*sT2*cA2*cT3*cA3*r4*sT4)
            +(sA1*sT2*sA2*sA3*r4*sT4)
            +(sA1*cT2*sT3*sA3*d4)
            +(sA1*sT2*cA2*cT3*sA3*d4)
            +(sA1*sT2*sA2*cA3*d4)
            +(sA1*cT2*r3*cT3)
            -(sA1*sT2*cA2*r3*sT3)
            +(sA1*sT2*sA2*d3)
            +(sA1*r2*cT2);
  j(2,2) = -(sA1*sT2*sT3*r4*cT4)
            +(sA1*cT2*cA2*cT3*r4*cT4)
            +(cA1*sA2*cT3*r4*cT4)
            -(sA1*sT2*cT3*cA3*r4*sT4)
            -(sA1*cT2*cA2*sT3*cA3*r4*sT4)
            -(cA1*sA2*sT3*cA3*r4*sT4)
            +(sA1*sT2*cT3*sA3*d4)
            +(sA1*cT2*cA2*sT3*sA3*d4)
            +(cA1*sA2*sT3*sA3*d4)
            -(sA1*sT2*r3*sT3)
            +(sA1*cT2*cA2*r3*cT3)
            +(cA1*sA2*r3*cT3);
  j(2,3) = -(sA1*sT2*cT3*r4*sT4)
            -(sA1*cT2*cA2*sT3*r4*sT4)
            -(cA1*sA2*sT3*r4*sT4)
            -(sA1*sT2*sT3*cA3*r4*cT4)
            +(sA1*cT2*cA2*cT3*cA3*r4*cT4)
            +(cA1*sA2*cT3*cA3*r4*cT4)
            -(sA1*cT2*sA2*sA3*r4*cT4)
            +(cA1*cA2*sA3*r4*cT4);
  return j;
}

MatrixXd createJacobian5DOF(vector<map<string, double>> dh) //TBD
{
  //double d1 = dh[0]["d"];
  //double d2 = dh[1]["d"];
  //double d3 = dh[2]["d"];
  //double d4 = dh[3]["d"];
  //double d5 = dh[4]["d"];
  //double r1 = dh[0]["r"];
  //double r2 = dh[1]["r"];
  //double r3 = dh[2]["r"];
  //double r4 = dh[3]["r"];
  //double r5 = dh[4]["r"];
  //double sT1 = sin(dh[0]["theta"]);
  //double sT2 = sin(dh[1]["theta"]);
  //double sT3 = sin(dh[2]["theta"]);
  //double sT4 = sin(dh[3]["theta"]);
  //double sT5 = sin(dh[4]["theta"]);
  //double cT1 = cos(dh[0]["theta"]);
  //double cT2 = cos(dh[1]["theta"]);
  //double cT3 = cos(dh[2]["theta"]);
  //double cT4 = cos(dh[3]["theta"]);
  //double cT5 = cos(dh[4]["theta"]);
  //double sA1 = sin(dh[0]["alpha"]);
  //double sA2 = sin(dh[1]["alpha"]);
  //double sA3 = sin(dh[2]["alpha"]);
  //double sA4 = sin(dh[3]["alpha"]);
  //double sA5 = sin(dh[4]["alpha"]);
  //double cA1 = cos(dh[0]["alpha"]);
  //double cA2 = cos(dh[1]["alpha"]);
  //double cA3 = cos(dh[2]["alpha"]);
  //double cA4 = cos(dh[3]["alpha"]);
  //double cA5 = cos(dh[4]["alpha"]);
  MatrixXd j(3,5);
  j(0,0) = 0.0;
  j(0,1) = 0.0;
  j(0,2) = 0.0;
  j(0,3) = 0.0;
  j(0,4) = 0.0;
  j(1,0) = 0.0;
  j(1,1) = 0.0;
  j(1,2) = 0.0;
  j(1,3) = 0.0;
  j(1,4) = 0.0;
  j(2,0) = 0.0;
  j(2,1) = 0.0;
  j(2,2) = 0.0;
  j(2,3) = 0.0;
  j(2,4) = 0.0;
  return j;
}

MatrixXd createJacobian6DOF(vector<map<string, double>> dh) //TBD
{
  //double d1 = dh[0]["d"];
  //double d2 = dh[1]["d"];
  //double d3 = dh[2]["d"];
  //double d4 = dh[3]["d"];
  //double d5 = dh[4]["d"];
  //double d6 = dh[5]["d"];
  //double r1 = dh[0]["r"];
  //double r2 = dh[1]["r"];
  //double r3 = dh[2]["r"];
  //double r4 = dh[3]["r"];
  //double r5 = dh[4]["r"];
  //double r6 = dh[5]["r"];
  //double sT1 = sin(dh[0]["theta"]);
  //double sT2 = sin(dh[1]["theta"]);
  //double sT3 = sin(dh[2]["theta"]);
  //double sT4 = sin(dh[3]["theta"]);
  //double sT5 = sin(dh[4]["theta"]);
  //double sT6 = sin(dh[5]["theta"]);
  //double cT1 = cos(dh[0]["theta"]);
  //double cT2 = cos(dh[1]["theta"]);
  //double cT3 = cos(dh[2]["theta"]);
  //double cT4 = cos(dh[3]["theta"]);
  //double cT5 = cos(dh[4]["theta"]);
  //double cT6 = cos(dh[5]["theta"]);
  //double sA1 = sin(dh[0]["alpha"]);
  //double sA2 = sin(dh[1]["alpha"]);
  //double sA3 = sin(dh[2]["alpha"]);
  //double sA4 = sin(dh[3]["alpha"]);
  //double sA5 = sin(dh[4]["alpha"]);
  //double sA6 = sin(dh[5]["alpha"]);
  //double cA1 = cos(dh[0]["alpha"]);
  //double cA2 = cos(dh[1]["alpha"]);
  //double cA3 = cos(dh[2]["alpha"]);
  //double cA4 = cos(dh[3]["alpha"]);
  //double cA5 = cos(dh[4]["alpha"]);
  //double cA6 = cos(dh[5]["alpha"]);
  MatrixXd j(3,6);
  j(0,0) = 0.0;
  j(0,1) = 0.0;
  j(0,2) = 0.0;
  j(0,3) = 0.0;
  j(0,4) = 0.0;
  j(0,5) = 0.0;
  j(1,0) = 0.0;
  j(1,1) = 0.0;
  j(1,2) = 0.0;
  j(1,3) = 0.0;
  j(1,4) = 0.0;
  j(1,5) = 0.0;
  j(2,0) = 0.0;
  j(2,1) = 0.0;
  j(2,2) = 0.0;
  j(2,3) = 0.0;
  j(2,4) = 0.0;
  j(2,5) = 0.0;
  return j;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
