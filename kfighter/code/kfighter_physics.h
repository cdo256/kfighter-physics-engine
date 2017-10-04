#if !defined(KFIGHTER_PHYSICS_H)
/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

global const int maxVerticesInPolygon = 16;

struct Polygon {
    int count;
    v2 center;
    v2 verts[maxVerticesInPolygon];
};

struct PhysicsRect {
    f32 w, h;
    v2 p;
    v2 v;
    v2 lastV;
    v2 accel;
    f32 angle;
    f32 angularVel;
    f32 mass;
    f32 momentOfInertia;
    bool fixed;
    bool enableFriction;

    u32 colour;
};

struct CollisionManifold {
    int count;
    v2 pos[2];
    v2 normal;
    f32 depth;
    PhysicsRect *r1, *r2;
};

struct PhysicsJoint {
    PhysicsRect* r1;
    PhysicsRect* r2;
    v2 relPos1, relPos2;
    f32 minTheta, maxTheta;
    f32 maxTorque;
    f32 targetAngVel;
    f32 targetAngle;

    f32 pidLastError;
    f32 pidIntegralTerm;
    
    bool enableMotor;
    bool enablePID;
    bool enable;
};

//NOTE: Collision islands are collections of objects typically
//connected by joints or other constraints that shouldn't collide with
//each other.
struct CollisionIsland {
    bool enable;
    
    PhysicsRect* rects;
    int count;
};

#define KFIGHTER_PHYSICS_H
#endif
