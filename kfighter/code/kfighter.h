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

#define internal static
#define local_persist static
#define global static

#define KILOBYTES(x) ((x)*1024LL)
#define MEGABYTES(x) (KILOBYTES(x)*1024LL)
#define GIGABYTES(x) (MEGABYTES(x)*1024LL)

#define min(x,y) ((x)<(y)?(x):(y))
#define max(x,y) ((x)>(y)?(x):(y))
#define bound(x,lower,upper) (max((lower),min((upper),(x))))

#include <stdint.h>

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef s8 b8;
typedef s16 b16;
typedef s32 b32;
typedef s64 b64;

typedef float f32;
typedef double f64;

global const f32 epsilon = 0.000001f;
global const f32 pi = 3.14152635389793238f;

inline f32 sqr(f32 x);

struct v2 {
    f32 x, y;
};

inline v2 V2(f32 x, f32 y);

inline v2 operator+(v2 vec);
inline v2 operator-(v2 vec);
inline v2 operator+(v2 lhs, v2 rhs);
inline v2 operator-(v2 lhs, v2 rhs);
inline v2 operator*(f32 scalar, v2 vec);
inline v2 operator*(v2 vec, f32 scalar);
inline v2 operator/(v2 vec, f32 scalar);

inline v2 &operator+=(v2 &lhs, v2 rhs);
inline v2 &operator-=(v2 &lhs, v2 rhs);
inline v2 &operator*=(v2 &lhs, f32 scalar);
inline v2 &operator/=(v2 &lhs, f32 scalar);

inline bool operator==(v2 lhs, v2 rhs);
inline bool operator!=(v2 lhs, v2 rhs);

inline f32 sqrmag(v2 vec);
inline f32 mag(v2 vec);
inline v2 norm(v2 vec);
inline v2 perp(v2 vec); // = rotate(vec,pi/2)
inline f32 dot(v2 vec1, v2 vec2);
inline f32 cross(v2 vec1, v2 vec2);
inline v2 rotate(v2 vec, f32 angle);

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

global const int maxVerticesInPolygon = 16;

struct Polygon {
    int count;
    v2 center;
    v2 verts[maxVerticesInPolygon];
};

struct CollisionInfo {
    v2 pos;
    v2 normal;
    f32 depth;
};

struct PhysicsParticle {
    v2 p;
    v2 v;
    f32 mass;
};

struct PhysicsRect {
    f32 w, h;
    v2 p;
    v2 v;
    f32 angle;
    f32 angularVel;
    f32 mass;
    f32 momentOfInertia;
    bool fixed;

    u32 colour;
    //int jointCount;
    //PhysicsJoint joints[4];
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

    bool enable;
};

struct PhysicsPlayer {
    f32 w, h;
    v2 p;
    v2 v;
    f32 angle;
    f32 angularVel;
    f32 mass;
    f32 momentOfInertia;
};

union PlayerSegments {
    PhysicsRect segments[11];
    struct {
        PhysicsRect head;
        PhysicsRect chest;
        PhysicsRect abdomin;
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

struct PlayerBody {
    PlayerSegments* segments;
};

global const int maxParticles = 10000;
global const int maxRects = 100;
global const int maxPlayers = 4;
global const int maxJoints = 50;
global const int maxCollsionManifolds = 100;

struct GameState {
    b32 isInitialised;
    s32 offsetX;
    s32 offsetY;

    int playerX, playerY;

    PlayerBody playerBody;

    int particleCount;
    PhysicsParticle particles[maxParticles];
    
    int rectCount;
    PhysicsRect rects[maxRects];

    //NOTE: Rects are stored in this order
    int fixedRectCount;
    int stringSegmentRectCount;
    int playerSegmentRectCount;
    int freeRectCount;

    int playerCount;
    PhysicsPlayer players[maxPlayers];

    int jointCount;
    PhysicsJoint joints[maxJoints];

    //NOTE: Segments are stored in this order
    int stringJointCount;
    int playerJointCount;
    
    v2 globalForce;
    v2 globalAccel;
    int globalAccelTimer;
    v2 globalVel;
    v2 globalPos;

    bool collision;
    bool debugPause;
    bool stringEnable; //hang string from top of screen

    //TODO: Use these
    int collisionManifoldCount;
    CollisionManifold collisionManifolds[maxCollsionManifolds];
};

#if KFIGHTER_SLOW
#define assert(expr) {if (!(expr)) *(int*)0 = 0;}
#else
#define assert(expr)
#endif

#define KFIGHTER_H
#endif
