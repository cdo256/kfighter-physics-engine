/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

#include "kfighter_util.h"

internal void seedRandomNumberGenerator(modified_(randomSeed) GameState* state) {
    state->randomSeed = (u32)(1103515245 * state->randomSeed + 12345);
}

internal s32 randIntBetween(GameState* state, s32 s, s32 e) {
    seedRandomNumberGenerator(state);
    return state->randomSeed % (e-s) + s;
}

//NOTE: Generates between 0 and 1
internal f32 rand(GameState* state) {
    seedRandomNumberGenerator(state);
    return state->randomSeed / (float)(0x100000000LL);
}

//NOTE: Decides if the button was depressed or released, doesn't
//capture single frame presses
internal bool wasDepressed(GameButtonState button) {
    return button.endedDown && (button.halfTransitionCount % 2 == 1);
}
internal bool wasReleased(GameButtonState button) {
    return !button.endedDown && button.halfTransitionCount;
}
internal bool wasTapped(GameButtonState button) {
    return wasDepressed(button) || button.halfTransitionCount >= 2;
}

internal void makeWalls(GameState* state, GameOffscreenBuffer* buffer) {
    CollisionIsland* floorIsland = &state->collisionIslandArr[state->collisionIslandCount++];
    CollisionIsland* ceilIsland = &state->collisionIslandArr[state->collisionIslandCount++];
    CollisionIsland* leftWallIsland = &state->collisionIslandArr[state->collisionIslandCount++];
    CollisionIsland* rightWallIsland = &state->collisionIslandArr[state->collisionIslandCount++];

    floorIsland->rectCount = ceilIsland->rectCount =
        leftWallIsland->rectCount = rightWallIsland->rectCount = 1;
    floorIsland->enable = ceilIsland->enable =
        leftWallIsland->enable = rightWallIsland->enable = true;
    PhysicsRect* r;
    for (int i = 0; i < 4; i++) {
        r = &state->rectArr[state->rectCount++];
        r->fixed = true;
        r->enableFriction = true;
        r->v = V2(0,0);
        r->lastV = V2(0,0);
        r->angularVel = 0;
        r->mass = FLT_MAX;
        r->momentOfInertia = FLT_MAX;
        r->angle = 0;
        r->colour = 0x00FFFFFF;
        if (i == 0) {
            r->w = (f32)buffer->width;
            r->h = 50;
            r->p.x = (f32)buffer->width/2.f;
            r->p.y = -r->h/2-.5f;
            floorIsland->rects = r;
        } else if (i == 1) {
            r->w = (f32)buffer->width;
            r->h = 50;
            r->p.x = (f32)buffer->width/2.f;
            r->p.y = (f32)buffer->height+r->h/2+.5f;
            ceilIsland->rects = r;
        } else if (i == 2) {
            r->w = 50;
            r->h = (f32)buffer->height;
            r->p.x = -r->w/2-.5f;
            r->p.y = (f32)buffer->height/2.f;
            leftWallIsland->rects = r;
        } else if (i == 3) {
            r->w = 50;
            r->h = (f32)buffer->height;
            r->p.x = (f32)buffer->width+r->w/2+.5f;
            r->p.y = (f32)buffer->height/2.f;
            rightWallIsland->rects = r;
        }
    }
}

