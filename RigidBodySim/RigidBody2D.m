//
//  RigidBody2D.m
//  RigidBodySim
//
//  Created by Adriano Papa on 6/23/14.
//  Copyright (c) 2014 Adriano Papa. All rights reserved.
//

#import "RigidBody2D.h"

@implementation RigidBody2D

- (id)initRigidBody2D
{
    self = [super init];
    
    if (self)
    {
        fMass = 100.0;
        fInertia = 500;
        fInertiaInverse = 1/fInertia;
        vPosition = [[Vector alloc] initWithZeros];
        fWidth = 10;
        fLenght = 20;
        fHeight = 5;
        fOrientation = 0;
        CD = [[Vector alloc] initWithX:-0.25*fLenght Y:0.0f Z:0.0f];
        CT = [[Vector alloc] initWithX:-0.5*fLenght Y:0.0 Z:0.0];
        CPT = [[Vector alloc] initWithX:0.5*fLenght Y:-0.5*fWidth Z:0.0];
        CST = [[Vector alloc] initWithX:0.5*fLenght Y:0.5*fWidth Z:0.0];
        
        fProjectedArea = (fLenght+fWidth)/2 * fHeight;
        fThrustForce = _THRUSTFORCE;
    }
    
    return self;
}

- (void)calcLoads
{
    Vector *Fb = [[Vector alloc] initWithZeros];
    Vector *Mb = [[Vector alloc] initWithZeros];
    Vector *Thrust = [[Vector alloc] initWithX:1.0 Y:0.0 Z:0.0];
    
    vForces.x = vForces.y = vForces.z = 0.0;
    [Thrust Mul:fThrustForce];
    
    Vector *vLocalVelocity = [[Vector alloc] initWithZeros];
    float fLocalSpeed;
    Vector *vDragVector = [[Vector alloc] initWithZeros];
    float fTmp;
    Vector *vResultant = [[Vector alloc] initWithZeros];
    Vector *vTmp;
    
    vTmp = [Vector Cross:vAngularVelocity _v2:CD];
    vLocalVelocity = [Vector Add:vVelocityBody _v2:vTmp];
    
    fLocalSpeed = [vLocalVelocity Magnitude];
    
}

@end
