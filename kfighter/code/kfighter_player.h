#if !defined(KFIGHTER_PLAYER_H)
/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

global const int playerSegmentCount = 11;
global const int playerJointCount = 10;

union PlayerSegments {
    PhysicsRect segments[playerSegmentCount];
    struct {
        PhysicsRect head;
        PhysicsRect chest;
        PhysicsRect abdomen;
        PhysicsRect lBicep;
        PhysicsRect lForearm;
        PhysicsRect rBicep;
        PhysicsRect rForearm;
        PhysicsRect lThigh;
        PhysicsRect lShin;
        PhysicsRect rThigh;
        PhysicsRect rShin;
    };
};

union PlayerJoints {
    PhysicsJoint joints[playerJointCount];
    struct {
        PhysicsJoint neck;
        PhysicsJoint back;
        PhysicsJoint lShoulder;
        PhysicsJoint lElbow;
        PhysicsJoint rShoulder;
        PhysicsJoint rElbow;
        PhysicsJoint lHip;
        PhysicsJoint lKnee;
        PhysicsJoint rHip;
        PhysicsJoint rKnee;
    };
};

struct PoseJoint {
    f32 angle;
    f32 applicationFactor; // 1 is apply fully, 0 is don't apply (used in interpolation)
};

struct PlayerPose {
    union {
        PoseJoint joints[playerJointCount];
        struct {
            PoseJoint neck;
            PoseJoint back;
            PoseJoint lShoulder;
            PoseJoint lElbow;
            PoseJoint rShoulder;
            PoseJoint rElbow;
            PoseJoint lHip;
            PoseJoint lKnee;
            PoseJoint rHip;
            PoseJoint rKnee;
        };
    };
};

enum Direction {DIRECTION_LEFT, DIRECTION_RIGHT};

struct Player {
    PlayerSegments* segments;
    PlayerJoints* joints;
    Direction direction;
    
    //TODO: The following is all legacy stuff and needs to go at some point
    PlayerPose* currentPose;
    PlayerPose *prevPose, *nextPose;
    f32 strideWheelRadius;
    f32 strideWheelAngle;
    
    bool lPunching, rPunching;
    f32 rPunchTimer, lPunchTimer;
    v2 lPunchTarget, rPunchTarget;
    f32 torque;
};

#define KFIGHTER_PLAYER_H
#endif
