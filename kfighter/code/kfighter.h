#if !defined(KFIGHTER_H)

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

//TODO: Can I simplify this? Is there more I need?
struct GameMouseButtonState {
	f32 lastTransitionX, lastTransitionY;
	s32 halfTransitionCount;
	b32 endedDown;
};

struct GameMouseInput {
	b32 enabled;
	GameMouseButtonState leftButton;
	GameMouseButtonState rightButton;
	GameMouseButtonState middleButton;
	f32 scrollStart, scrollEnd;
	f32 startX, startY;
	f32 endX, endY;

	b32 isCaptured;
};

struct GameKeyState {
	s8 halfTransitionCount : 6;
	b8 endedDown : 1;
	b8 seen : 1; //NOTE: This stays cleared until a key press or
				 //release is seen
};

//TODO: Do we have the problem of thinking that keys are still down
//when the window loses focus?
struct GameKeyboardInput {
	b32 enabled;
	GameKeyState keys[256];
};

struct GameInput {
	GameControllerInput controllers[5];
	GameMouseInput mouse;
	GameKeyboardInput keyboard;
};

struct GameMemory {
	u64 permanentStorageSize;
	void* permanentStorage;

	u64 transientStorageSize;
	void* transientStorage;
};

#define GAME_UPDATE_AND_RENDER(name)						 \
	void name(										   \
		f32 dt,										  \
		u32 seed,										\
		modified_descendent GameMemory* memory,		  \
		in GameInput* input,							 \
		modified_descendent GameOffscreenBuffer* buffer)
typedef GAME_UPDATE_AND_RENDER(game_update_and_render);
GAME_UPDATE_AND_RENDER(gameUpdateAndRenderStub) {}
//global game_update_and_render* gameUpdateAndRender_ = gameUpdateAndRenderStub;
//#define gameUpdateAndRender gameUpdateAndRender_

extern "C" GAME_UPDATE_AND_RENDER(gameUpdateAndRender);

//NOTE: Platform independent declarations

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

struct PhysicsVariables {
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

	v2 globalForce;
	v2 globalAccel;
};

struct GameState {
	b32 isInitialised;

	u32 playerCount;
	Player playerArr[4];
	u32 objCount;
	PhysicsObj objArr[100];
	u32 jointCount;
	PhysicsJoint jointArr[50];
	u32 collisionIslandCount;
	CollisionIsland collisionIslandArr[20];
	u32 collisionManifoldCount;
	CollisionManifold collisionManifoldArr[200];
	u32 poseCount;
	PlayerPose poseArr[PLAYER_POSE_ENUM_COUNT];

	u32 randomSeed;

	f32 metersToPixels;

	PhysicsVariables physicsVariables;

	u32 backgroundColour;
};

#if KFIGHTER_SLOW
#define assert(expr) (expr) ? (void)0 : (void)(*(int*)0 = 0)
#else
#define assert(expr) (void)0
#endif

#define KFIGHTER_H
#endif
