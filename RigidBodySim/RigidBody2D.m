//
//  RigidBody2D.m
//  RigidBodySim
//
//  Created by Adriano Papa on 6/23/14.
//  Copyright (c) 2014 Adriano Papa. All rights reserved.
//

#import "RigidBody2D.h"

CGFloat DegreesToRadians(CGFloat degrees) {return degrees * M_PI / 180;};
CGFloat RadiansToDegrees(CGFloat radians) {return radians * 180/M_PI;};

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
    
    if (fLocalSpeed > tol)
    {
        [vLocalVelocity Normalize];
        
        vDragVector.x = -vLocalVelocity.x;
        vDragVector.y = -vLocalVelocity.y;
        vDragVector.z = -vLocalVelocity.z;
        
        fTmp = 0.5f * _AIRDENSITY * fLocalSpeed * fLocalSpeed * fProjectedArea;
        
        Vector *_v = [[Vector alloc] initWithX:vDragVector.x Y:vDragVector.y Z:vDragVector.z];
        [_v Mul:_LINEARDRAGCOEFFICIENT*fTmp];
        vResultant.x = _v.x;
        vResultant.y = _v.y;
        vResultant.z = _v.z;
        
        [Fb Add:vResultant];
        
        vTmp = [Vector Cross:CD _v2:vResultant];
        [Mb Add:vTmp];
    }
    
    [Fb Add:vPThrust];
    
    vTmp = [Vector Cross:CPT _v2:vPThrust];
    [Mb Add:vTmp];
    [Fb Add:vPThrust];
    
    vForces = [self vRotate2DWithAngle:fOrientation appliedTo:Fb];
    
    [vMoment Add:Mb];
}

- (void)updateBodyEuler:(double)dt
{
    Vector *a = [[Vector alloc] initWithZeros];
    Vector *dv = [[Vector alloc] initWithZeros];
    Vector *ds = [[Vector alloc] initWithZeros];
    float aa,dav,dr;
    
    [self calcLoads];
    
    a = [Vector Div:vForces scalar:fMass];
    
    dv = [Vector Mul:a scalar:dt];
    [vVelocity Add:dv];
    
    ds = [Vector Mul:vVelocity scalar:dt];
    [vPosition Add:ds];
    
    aa = vMoment.z / fInertia;
    dav = aa*dt;
    
    vAngularVelocity.z += dav;
    
    dr = RadiansToDegrees(vAngularVelocity.z * dt);
    fOrientation += dr;
    
    fSpeed = [vVelocity Magnitude];
    vVelocityBody = [self vRotate2DWithAngle:-fOrientation appliedTo:vVelocity];
}

- (Vector*)vRotate2DWithAngle:(float)angleInDegree appliedTo:(Vector*)u
{
    Vector *vr = [[Vector alloc] initWithZeros];
    
    CGAffineTransform _rotate = CGAffineTransformMakeRotation(DegreesToRadians(angleInDegree));
    
    CGPoint _point = CGPointMake(u.x, u.y);
    
    CGPoint _new_point = CGPointApplyAffineTransform(_point, _rotate);
    
    vr.x = _new_point.x;
    vr.y = _new_point.y;
    
    return vr;
}

@end
