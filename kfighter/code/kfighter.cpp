/* TODO: Lots to do before I have a game
   Physics:
	- Multi point collision
	- Static/dynamic friction
	- Collision boucing (bias)
	- Capsule collision detection
	- Is torque being applied correctly?
   Motion Intelligence:
	- Standing up
	- Target positions and velocity
	- Split by component (eg. legs, arms, torso for punching)
	- attempt to maximize velocity within time t
	- Stepping
	- Cancel target when hit or target unreachable
	- Base stance to retract to
	- Ducking and jumping
	- Punching and kicking
	- Walking and running
	- Flips
	- Do players need feet?
   Game:
	- Scoring
	- KOs
   Refactor:
	- Bring out control variables into the state
	- Bring common routines into functions
	- Group like code
	- Remove dead #if 0's
	- Move other #if 0's to own function
	- Memory arenas
	- First optimisation pass
	- Get the poses out of the GameState
   Visuals:
	- Nicer looking players?
	- Moving camera?
	- Visual objects separate from physics objects
   User Interface:
	- Use mouse to drag parts of the world around
	- Send keyboard past platform layer
	- Automatic resizing game environment whenever window is resized
	- User controlled player
   Debug:
	- Logging system for our code
	- Introspection?
   Admin:
	- Changelog?
 */

#include "kfighter.h"
#include "kfighter_util.cpp"
#include "kfighter_maths.cpp"
#include "kfighter_geometry.cpp"
#include "kfighter_physics.cpp"
#include "kfighter_render.cpp"
#include "kfighter_player.cpp"


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////// GAME UPDATE AND RENDER //////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

GAME_UPDATE_AND_RENDER(gameUpdateAndRender) {
	assert(sizeof(GameState) <= memory->permanentStorageSize);
	GameState* state = (GameState*)memory->permanentStorage;

	//NOTE: Force dt for debugging only
#if KFIGHTER_INTERNAL
	dt = 0.033f;
#endif


	// ---- INIT ----

	if (!state->isInitialised) {
	    state->randomSeed = seed;

	    state->playerCount = 0;
	    state->rectCount = 0;
	    state->jointCount = 0;
	    state->collisionIslandCount = 0;
	    state->collisionManifoldCount = 0;
	    state->poseCount = PLAYER_POSE_ENUM_COUNT;

	    state->metersToPixels = 175.f;
	    setPhysicsConstants(&state->physicsVariables);

	    state->backgroundColour = 0x00FFFFFF;//0x003B80FF;

	    makeWalls(state, buffer);

	    for (int i = 0; i < 6; i++) {
	        PhysicsRect* r = GET_NEXT_ARRAY_ELEM_WITH_FAIL(state->rect);
	        CollisionIsland* ci = GET_NEXT_ARRAY_ELEM_WITH_FAIL(state->collisionIsland);
	        ci->enable = true;
	        ci->rectCount = 1;
	        ci->rects = r;
	        r->w = r->h = 100;
	        r->p = V2(buffer->width/6.f + 10*rand(state), i*r->h + r->h/2);
	        r->v = V2(0,0);
	        r->lastV = r->v;
	        r->angle = 0;
	        r->angularVel = 0;
	        r->fixed = false;
	        r->enableFriction = true;
	        r->colour = 0x00000000;
	        computeMassAndMomentOfInertia(r, 0.01f);
	    }

	    state->playerCount = 2;
	    makePlayer(state, &state->playerArr[0], buffer);
	    makePlayer(state, &state->playerArr[1], buffer);
	    makePlayerPoses(state);
	    assert(ARRAY_SIZE_CHECK(state->player));

	    state->isInitialised = true;
	}


	// ---- INPUT ----

	GameControllerInput* controller = &input->controllers[0];

	if (wasTapped(controller->aButton)) {
	    for (int i = 0; i < state->playerCount; i++) {
	        Player* player = &state->playerArr[i];
	        player->currentPose++;
	        if (player->currentPose >= state->poseCount)
	            player->currentPose = -1;
	    }
	}

	f32 keyboardAccelScalar = 2000.f;

	v2 keyboardAccel = {};
	if (controller->up.endedDown)
	    keyboardAccel.y -= keyboardAccelScalar;
	if (controller->down.endedDown)
	    keyboardAccel.y += keyboardAccelScalar;
	if (controller->left.endedDown)
	    keyboardAccel.x += keyboardAccelScalar;
	if (controller->right.endedDown)
	    keyboardAccel.x -= keyboardAccelScalar;


	// ---- UPDATE AND RENDER ----

	for (int i = 0; i < state->rectCount; i++) {
	    PhysicsRect* r = &state->rectArr[i];
	    r->accel = (r->v - r->lastV) / dt;
	    r->lastV = r->v;
	}

	for (int j = 0; j < state->playerCount; j++) {
	    for (int i = 0; i < 10; i++) {
	        state->playerArr[j].joints->joints[i].targetAngVel
	            = state->physicsVariables.motorTargetAngVel;
	    }
	}

	state->physicsVariables.globalForce = {};
	state->physicsVariables.globalAccel = V2(0,-1500.f);
	state->physicsVariables.globalAccel += keyboardAccel;

	renderBackground(buffer, state->backgroundColour);


#if 1 //NOTE: Test render so we can see debug overlays
	//TODO: This is really janky, surely there's a better way.
	// --- RENDER ---
	for (int i = 0; i < state->rectCount; i++) {
	    PhysicsRect* r = &state->rectArr[i];
	    renderRectangle(buffer, r, r->colour);
	}
	for (int i = 0; i < state->playerCount; i++) { // Render player to get rects in correct order
	    PhysicsRect* r;
	    PlayerSegments* segs = state->playerArr[i].segments;

	    r = &segs->lShin;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->lThigh;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->lBicep;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->lForearm;
	    renderRectangle(buffer, r, r->colour);

	    r = &segs->head;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->chest;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->abdomen;
	    renderRectangle(buffer, r, r->colour);

	    r = &segs->rShin;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->rThigh;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->rBicep;
	    renderRectangle(buffer, r, r->colour);
	    r = &segs->rForearm;
	    renderRectangle(buffer, r, r->colour);
	}
#endif

	for (int i = 0; i < state->playerCount; i++)
	    updatePlayer(&state->playerArr[i], state->poseArr, dt);

	// --- SIMULATE ---

	//TODO: Vary iterCount based on load

	//TODO: Do coarse collision check once to create the (close but
	//possibly not colliding) manifolds then only test each manifold
	//for collisions
	int iterCount = 30;
	f32 h = dt/iterCount;
	for (int iter = 0; iter < iterCount; iter++) {
	    // -- UPDATE POSTION --
	    for (int i = 0; i < state->rectCount; i++) {
	        PhysicsRect* r = &state->rectArr[i];

	        if (r->fixed) {
	            r->v = V2(0,0);
	            r->angularVel = 0;
	            r->mass = FLT_MAX;
	            r->momentOfInertia = FLT_MAX;
	        } else {
	            r->v += h*state->physicsVariables.globalForce/r->mass;
	            r->v += h*state->physicsVariables.globalAccel;
	        }

	        r->p += h * r->v;
	        r->angle += h * r->angularVel;
	        r->angle = normAngle(r->angle);
	    }

	    // -- COLLIDE --
	    if (state->physicsVariables.enableCollision) {
	        state->collisionManifoldCount = 0;

	        //TODO: Is this too slow? Is there a better way to do this
	        //where we only consider objects that are close to
	        //colliding?
	        for (int i1 = 0; i1 < state->collisionIslandCount; i1++) {
	            CollisionIsland* ci1 = &state->collisionIslandArr[i1];
	            if (!ci1->enable) continue;
	            for (int i2 = 0; i2 < i1; i2++) {
	                CollisionIsland* ci2 = &state->collisionIslandArr[i2];
	                if (!ci2->enable) continue;
	                for (int ri1 = 0; ri1 < ci1->rectCount; ri1++) {
	                    PhysicsRect* r1 = &ci1->rects[ri1];
	                    Polygon p1; computeRectVertices(r1,&p1);
	                    for (int ri2 = 0; ri2 < ci2->rectCount; ri2++) {
	                        PhysicsRect* r2 = &ci2->rects[ri2];
	                        Polygon p2; computeRectVertices(r2, &p2);
	                        if (r1->fixed && r2->fixed) continue;
	                        CollisionManifold collisionManifold;
	                        if (doPolygonsIntersect(
	                                &p1,&p2,&collisionManifold)) {
	                            if (ARRAY_APPEND_CHECK(state->collisionManifold)) {
	                                collisionManifold.r1 = r1;
	                                collisionManifold.r2 = r2;
	                                APPEND_TO_ARRAY_WITHOUT_CHECK(
	                                    state->collisionManifold,
	                                    collisionManifold)
	                            } else {
	                                //NOTE: This should only occur on
	                                //heavy load frames. Nothing bad
	                                //happens if we ignore collisions
	                                //in this case

	                                //TODO: Log that we've run out of
	                                //manifolds
	                            }
	                        }
	                    }
	                }
	            }
	        }

	        for (int i = 0; i < state->collisionManifoldCount; i++) {
	            resolveCollision(
	                &state->physicsVariables,
	                h,
	                &state->collisionManifoldArr[i]);
	        }
	    }

	    // -- RESOLVE JOINT CONSTRAINTS --
	    for (int i = 0; i < state->jointCount; i++) {
	        resolveJointConstraint(
	            &state->physicsVariables,
	            &state->jointArr[i],
	            h);
	    }
	}

#if 0 //NOTE: actual render position, just need to move this for a test
	// --- RENDER ---
	for (int i = 0; i < state->rectCount; i++) {
	    PhysicsRect* r = &state->rectArr[i];
	    renderRectangle(buffer, r, r->colour);
	}
#endif
}
