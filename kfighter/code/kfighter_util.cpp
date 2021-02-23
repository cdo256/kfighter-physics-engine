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
	CollisionIsland* island = GET_NEXT_ARRAY_ELEM_WITH_FAIL(state->collisionIsland);
	island->rectCount = 4;
	island->enable = true;
	island->rects = &state->rectArr[state->rectCount];
	PhysicsRect* r;
	for (int i = 0; i < 4; i++) {
		r = GET_NEXT_ARRAY_ELEM_WITH_FAIL(state->rect);
		r->fixed = true;
		r->enableFriction = true;
		r->v = V2(0,0);
		r->lastV = V2(0,0);
		r->angularVel = 0;
		r->mass = FLT_MAX;
		r->momentOfInertia = FLT_MAX;
		r->angle = 0;
		r->colour = 0x00000000;
		if (i == 0) {
			r->w = (f32)buffer->width;
			r->h = 50;
			r->p.x = (f32)buffer->width/2.f;
			r->p.y = -r->h/2-.5f;
		} else if (i == 1) {
			r->w = (f32)buffer->width;
			r->h = 50;
			r->p.x = (f32)buffer->width/2.f;
			r->p.y = (f32)buffer->height+r->h/2+.5f;
		} else if (i == 2) {
			r->w = 50;
			r->h = (f32)buffer->height;
			r->p.x = -r->w/2-.5f;
			r->p.y = (f32)buffer->height/2.f;
		} else if (i == 3) {
			r->w = 50;
			r->h = (f32)buffer->height;
			r->p.x = (f32)buffer->width+r->w/2+.5f;
			r->p.y = (f32)buffer->height/2.f;
		}
	}
}

