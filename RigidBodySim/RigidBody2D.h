//
//  RigidBody2D.h
//  RigidBodySim
//
//  Created by Adriano Papa on 6/23/14.
//  Copyright (c) 2014 Adriano Papa. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "Vector.h"

#define _THRUSTFORCE    5.0f
#define _MAXTHRUST      10.f
#define _MINTHRUST      0.0f
#define _DTTHRUST       0.001f
#define _STEERINGFORCE  3.0f
#define _LINEARDRAGCOEFFICIENT  1.25f
#define _AIRDENSITY     1.204f

@interface RigidBody2D : NSObject
{
    float fMass;
    float fInertia;
    float fInertiaInverse;
    Vector *vPosition;
    Vector *vVelocity;
    Vector *vVelocityBody;
    Vector *vAngularVelocity;
    float fSpeed;
    float fOrientation;
    Vector *vForces;
    Vector *vMoment;
    float fThrustForce;
    Vector *vPThrust;
    Vector *vSThrust;
    float fWidth;
    float fLenght;
    float fHeight;
    Vector *CD;
    Vector *CT;
    Vector *CPT;
    Vector *CST;
    float fProjectedArea;
}

- (id)initRigidBody2D;
- (void)calcLoads;

@end
