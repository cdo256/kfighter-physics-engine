#if !defined(KFIGHTER_H)
/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

/* NOTE:

   KFIGHTER_INTERNAL:
    0 - Build for public release
    1 - Build for developer only

   KFIGHTER_SLOW:
    0 - No slow code allowed
    1 - Slow(debugging) code welcome
*/

//NOTE: Platform interface

#include "kfighter_global.h"
#include "kfighter_maths.h"
#include "kfighter_physics.h"
#include "kfighter_player.h"

struct GameOffscreenBuffer {
    void* bitmapMemory;
    u32 width;
    u32 height;
    u32 pitch;
    u32 bytesPerPixel;
};

struct GameButtonState {
    s32 halfTransitionCount;
    b32 endedDown;
};

struct GameStickState {
    f32 startX;
    f32 startY;
    f32 endX;
    f32 endY;
    f32 minX;
    f32 minY;
    f32 maxX;
    f32 maxY;
};

struct GameControllerInput {
    b32 isAnalog;
    
    GameStickState lStick;
    GameStickState rStick;
    
    union {
        GameButtonState buttons[12];
        struct {
            GameButtonState up;
            GameButtonState down;
            GameButtonState left;
            GameButtonState right;
            GameButtonState start;
            GameButtonState back;
            GameButtonState lShoulder;
            GameButtonState rShoulder;
            GameButtonState aButton;
            GameButtonState bButton;
            GameButtonState xButton;
            GameButtonState yButton;
        };
    };
};

struct GameInput {
    GameControllerInput controllers[5];
};

struct GameMemory {
    u64 permanentStorageSize;
    void* permanentStorage;
    
    u64 transientStorageSize;
    void* transientStorage;
};

#define GAME_UPDATE_AND_RENDER(name) \
    void name(                       \
        f32 dt,                      \
        u32 seed,                    \
        GameMemory* memory,          \
        GameInput* input,            \
        GameOffscreenBuffer* buffer)
typedef GAME_UPDATE_AND_RENDER(game_update_and_render);
GAME_UPDATE_AND_RENDER(gameUpdateAndRenderStub) {}
//global game_update_and_render* gameUpdateAndRender_ = gameUpdateAndRenderStub;
//#define gameUpdateAndRender gameUpdateAndRender_

GAME_UPDATE_AND_RENDER(gameUpdateAndRender);

//NOTE: Platform independent declarations

#define arrayCount(array) (sizeof(array) / sizeof((array)[0]))

global const int maxRects = 100;
global const int maxJoints = 50;
global const int maxCollsionManifolds = 100;
global const int maxCollisionIslands = 20;
global const int maxPoses = 20;
global const int maxPlayers = 4;

struct GameState {
    b32 isInitialised;
    
    int playerCount;
    Player players[maxPlayers];

    int poseCount;
    PlayerPose poses[maxPoses];
    PlayerPose* defaultPose;
    PlayerPose* ballPose;
    PlayerPose* readyPose;
    PlayerPose* punchPrepPose;
    PlayerPose* punchExtendPose;
    
    PlayerPose* runningLPassPose;
    PlayerPose* runningLReachPose;
    PlayerPose* runningRPassPose;
    PlayerPose* runningRReachPose;    
    PlayerPose* walkingLPassPose;
    PlayerPose* walkingLReachPose;
    PlayerPose* walkingRPassPose;
    PlayerPose* walkingRReachPose;

    int rectCount;
    PhysicsRect rects[maxRects];

    int collisionIslandCount;
    CollisionIsland collisionIslands[maxCollisionIslands];

    int jointCount;
    PhysicsJoint joints[maxJoints];
    
    v2 globalForce;
    v2 globalAccel;
    int globalAccelTimer;
    v2 globalVel;
    v2 globalPos;

    //TODO: Use these
    int collisionManifoldCount;
    CollisionManifold collisionManifolds[maxCollsionManifolds];
    
    //TODO: Is this random enough?
    u32 randomSeed;

    bool enableJoints;
    bool enableCollision;
    bool enableFriction;
    bool enableMotor;
    bool enablePIDJoints;
    bool enableRotationalConstraints;
    f32 frictionCoef;
    f32 jointFrictionCoef;
    f32 jointPositionalBiasCoef;

    f32 maxMotorTorque;
    f32 motorTargetAngVel;

    u32 backgroundColour;
};

#if KFIGHTER_SLOW
#define assert(expr) {if (!(expr)) *(int*)0 = 0;}
#else
#define assert(expr)
#endif

#define KFIGHTER_H
#endif
