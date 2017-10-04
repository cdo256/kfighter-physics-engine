/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

/* TODO: Lots to do before I have a game
   Physics:
    - Multi point collision
    - Joint friction
    - Static friction
    - Collision boucing (bias)
    - Capsule collision detection
    - Is torque being applied correctly?
   Motion Intelligence:
    - Target positions and velocity
    - Split by component (eg. legs, arms, torso for punching)
    - attempt to maximize velocity within time t
    - Stepping
    - Cancel target when hit or target unreachable
    - Base stance to retract to
    - Ducking and jumping
    - Punching and kicking
    - Walking and running
    - Standing up
    - Flips
   Game:
    - Other player
   Refactor:
    - Bring out control variables into the state
    - Bring common routines into functions
    - Group like code
    - Remove dead #if 0's
    - Move other #if 0's to own function
    - Memory arenas
 */

#include "kfighter.h"
#include "kfighter_maths.cpp"
#include "kfighter_geometry.cpp"
#include "kfighter_physics.cpp"

internal void renderBackground(GameState* state, GameOffscreenBuffer* buffer) {
    for (u32 y = 0; y < buffer->height; y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (u32 x = 0; x < buffer->width; x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            *((u32*)pixel) = state->backgroundColour;
        }
    }
}

internal void renderPoint(GameOffscreenBuffer* buffer, v2 pos, int size) {
    int xPos = (int)pos.x;
    int yPos = (int)pos.y;
    //x = bound(x,0,buffer->width);
    //y = bound(y,0,buffer->height);
    size /= 2;
    
    for (s32 y = max(0,yPos-size);
         y < min(yPos+size, (s32)buffer->height);
         y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (s32 x = max(0, xPos-size);
             x < min(xPos+size, (s32)buffer->width);
             x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            *((u32*)pixel) = 0x00FFFFFF;
        }
    }
}

internal void renderRectangle(GameOffscreenBuffer* buffer, PhysicsRect* r, u32 colour) {
    Polygon pRect;
    computeRectVertices(r, &pRect);
    v2* verts = pRect.verts;
    
    // Compute directions pointing inwards
    v2 dirs[4];
    for (int i=0;i<4;i++)
        dirs[i] = perp(verts[(i+1)%4] - verts[i]);

    int bottom = (int)buffer->height;
    int top = 0;
    int right = 0;
    int left = (int)buffer->width;
    for (int i=0;i<4;i++) {
        top = max(top,(int)verts[i].y);
        bottom = min(bottom,(int)verts[i].y);
        left = min(left,(int)verts[i].x);
        right = max(right,(int)verts[i].x);
    }
    top = bound(top,0,(int)buffer->height);
    bottom = bound(bottom,0,(int)buffer->height);
    left = bound(left,0,(int)buffer->width);
    right = bound(right,0,(int)buffer->width);
    
    for (int y = bottom; y < top; y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (int x = left; x < right; x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            v2 p = V2((f32)x,(f32)y);
            v2 topLeft = p - verts[1];
            v2 bottomRight = p - verts[3];
            if (dot(topLeft,dirs[0])>0 &&
                dot(topLeft,dirs[1])>0 &&
                dot(bottomRight,dirs[2])>0 &&
                dot(bottomRight,dirs[3])>0) {
            
                *((u32*)pixel) = colour;
            }
        }
    }
}

internal void seedRandomNumberGenerator(GameState* state) {
    state->randomSeed = (u32)(1103515245 * state->randomSeed + 12345);
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
    CollisionIsland* floorIsland = &state->collisionIslands[state->collisionIslandCount++];
    CollisionIsland* ceilIsland = &state->collisionIslands[state->collisionIslandCount++];
    CollisionIsland* leftWallIsland = &state->collisionIslands[state->collisionIslandCount++];
    CollisionIsland* rightWallIsland = &state->collisionIslands[state->collisionIslandCount++];

    floorIsland->count = ceilIsland->count =
        leftWallIsland->count = rightWallIsland->count = 1;
    floorIsland->enable = ceilIsland->enable =
        leftWallIsland->enable = rightWallIsland->enable = true;
    PhysicsRect* r;
    for (int i = 0; i < 4; i++) {
        r = &state->rects[state->rectCount++];
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
            r->p.y = -r->h/2;
            floorIsland->rects = r;
        } else if (i == 1) {
            r->w = (f32)buffer->width;
            r->h = 50;
            r->p.x = (f32)buffer->width/2.f;
            r->p.y = (f32)buffer->height+r->h/2;
            ceilIsland->rects = r;
        } else if (i == 2) {
            r->w = 50;
            r->h = (f32)buffer->height;
            r->p.x = -r->w/2;
            r->p.y = (f32)buffer->height/2.f;
            leftWallIsland->rects = r;
        } else if (i == 3) {
            r->w = 50;
            r->h = (f32)buffer->height;
            r->p.x = (f32)buffer->width+r->w/2;
            r->p.y = (f32)buffer->height/2.f;
            rightWallIsland->rects = r;
        }
    }
}

internal void makePlayer(GameState* state, Player* pl, GameOffscreenBuffer* buffer) {

    PhysicsRect* playerRect = &state->rects[state->rectCount++];
    pl->playerRect = playerRect;
    CollisionIsland* ci = &state->collisionIslands[state->collisionIslandCount++];
    ci->enable = true;
    ci->count = 1;
    ci->rects = playerRect;
    playerRect->w = 120;
    playerRect->h = 360;//300;
    playerRect->p = V2(buffer->width/2.f, buffer->height/2.f);
    playerRect->v = V2(0,0);
    playerRect->lastV = playerRect->v;
    playerRect->angle = 0;
    playerRect->angularVel = 0;
    playerRect->fixed = false;
    playerRect->enableFriction = false;
    playerRect->colour = state->backgroundColour;
    computeMassAndMomentOfInertia(playerRect);

    ci = &state->collisionIslands[state->collisionIslandCount++];
    ci->enable = false;
    ci->count = 11;
    ci->rects = &state->rects[state->rectCount];
    
    pl->segments = (PlayerSegments*)ci->rects;
    PhysicsRect* r;
    for (int i = 0; i < ci->count; i++) {
        r = &state->rects[state->rectCount++];
        r->fixed = false;
        r->enableFriction = true;
        r->p.x = rand(state)*buffer->width;
        r->p.y = rand(state)*buffer->height;
        r->v.x = (rand(state)-0.5f)*20.f;
        r->v.y = (rand(state)-0.5f)*20.f;
        r->lastV = r->v;
        r->angle = (rand(state)*2*pi);
        r->angularVel = (rand(state)*pi/4.f);
        if (pl == &state->players[0]) {
            r->colour = 0x00FF0000;
        } else {
            r->colour = 0x000000FF;
        }

        if (i == 0) { //head
            r->w = 60.f;
            r->h = 60.f;
        } else { //all others
            r->w = 20.f;
            r->h = 90.f;

            if (i == 3 || i == 4 || i == 7 || i == 8) {
                if (pl == &state->players[0]) {
                    r->colour = 0x00B00000;
                } else {
                    r->colour = 0x000000B0;
                }
            }
            
            //  0 - head
            //  1 - chest
            //  2 - abdomin
            //  3 - left bicep
            //  4 - left forearm
            //  5 - right bicep
            //  6 - right forearm
            //  7 - left thigh
            //  8 - left shin
            //  9 - right thigh
            // 10 - right shin
        }

        r->mass = r->w*r->h*0.01f;
        r->momentOfInertia = r->mass *
            (sqr(r->w)+sqr(r->h))/12.f;
    }

    // --- MAKE JOINTS ---
    PlayerSegments* seg = pl->segments;
    pl->joints = (PlayerJoints*)&state->joints[state->jointCount];
    state->jointCount += 10;
    
#define MAKE_JOINT(joint, part1, part2, min, max, vOffsetFactor) \
    {                                                            \
        PhysicsJoint* j = &pl->joints->joint;                  \
        j->r1 = &seg->part1;                                     \
        j->r2 = &seg->part2;                                     \
        j->relPos1 = -V2(                                        \
            0,                                                   \
            j->r1->h/2*(vOffsetFactor) - j->r1->w/2);            \
        j->relPos2 = +V2(0, j->r2->h/2 - j->r2->w/2);            \
        /*j->minTheta = (min)*pi;*/                              \
        /*j->maxTheta = (max)*pi;*/                              \
        /*NOTE: removing joint limits for the time being*/       \
        j->minTheta = -pi; j->maxTheta = pi;                     \
        j->targetAngVel = 0;                                     \
        j->targetAngle = 0;                                      \
        j->pidIntegralTerm = 0;                                  \
        j->enableMotor = false;                                  \
        j->enablePID = false;                                    \
        j->enable = true;                                        \
    }
    
    MAKE_JOINT(neck,      head,    chest,    -0.30f, 0.40f, 5.f/3.f);
    MAKE_JOINT(back,      chest,   abdomen,  -0.05f, 0.20f, 1);
    MAKE_JOINT(lShoulder, chest,   lBicep,   -0.70f, 0.90f, -0.0f);
    MAKE_JOINT(lElbow,    lBicep,  lForearm, -0.00f, 0.80f, 1);
    MAKE_JOINT(rShoulder, chest,   rBicep,   -0.70f, 0.90f, -0.0f);
    MAKE_JOINT(rElbow,    rBicep,  rForearm, -0.00f, 0.80f, 1);
    MAKE_JOINT(lHip,      abdomen, lThigh,   -0.30f, 0.60f, 1);
    MAKE_JOINT(lKnee,     lThigh,  lShin,    -0.70f, 0.00f, 1);
    MAKE_JOINT(rHip,      abdomen, rThigh,   -0.30f, 0.60f, 1);
    MAKE_JOINT(rKnee,     rThigh,  rShin,    -0.70f, 0.00f, 1);

#undef MAKE_JOINT

    // --- ADDITIONAL VARIABLES ---
    pl->currentPose = 0;//state->runningLReachPose;
    pl->torque = 0;
    pl->rPunching = false;
    pl->lPunching = false;
    pl->strideWheelRadius = 75.f;
    pl->strideWheelAngle = 0.f;
    pl->prevPose = state->runningLPassPose;
    pl->nextPose = state->runningLReachPose;
    pl->direction = DIRECTION_RIGHT;
}

internal PlayerPose lrFlipPlayerPose(PlayerPose* p) {
    PlayerPose result = *p;
    
    result.lShoulder = p->rShoulder;
    result.lElbow = p->rElbow;
    result.rShoulder = p->lShoulder;
    result.rElbow = p->lElbow;

    result.lHip = p->rHip;
    result.lKnee = p->rKnee;
    result.rHip = p->lHip;
    result.rKnee = p->lKnee;

    return result;
}

internal void makePlayerPoses(GameState* state) {
    // Default pose
    PlayerPose* pose = state->defaultPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].angle = 0.f;
        pose->joints[i].applicationFactor = 1.f;
    }

    // Ball pose
    pose = state->ballPose;
    Player* pl = &state->players[0];
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].angle = pl->joints->joints[i].maxTheta;
        pose->joints[i].applicationFactor = 1.f;
    }
    pose->lShoulder.angle = 0.1f*pi;
    pose->rShoulder.angle = 0.1f*pi;
    pose->lKnee.angle = pl->joints->lKnee.minTheta;
    pose->rKnee.angle = pl->joints->rKnee.minTheta;

    // Ready pose
    pose = state->readyPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].applicationFactor = 1.f;
    }
    
    pose->back.angle = 0.f;
    pose->neck.angle = 0.f;
    
    pose->lShoulder.angle = pi/5.f;
    pose->lElbow.angle = pi/2.f;
    pose->rShoulder.angle = -pi/5.f;
    pose->rElbow.angle = 3.f*pi/4.f;

    pose->lHip.angle = 4*pi/12.f;
    pose->lKnee.angle = -pi/4.f;
    pose->rHip.angle = -pi/12.f;
    pose->rKnee.angle = -pi/4.f;

    // Punch pose
#if 0 // old punch pose for free ragdolls
    pose = state->punchPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].applicationFactor = 1.f;
    }
    pose->back.angle = 0.f;
    pose->neck.angle = 0.f;
    
    pose->lShoulder.angle = -pi/5.f;
    pose->lElbow.angle = pi/4.f;
    pose->rShoulder.angle = pi/2.f + pi/5.f;
    pose->rElbow.angle = 0.f;

    pose->lHip.angle = 4*pi/12.f + pi/5.f;// - pi/8.f;
    pose->lKnee.angle = -3*pi/8.f;
    pose->rHip.angle = 0.f;//-pi/12.f + pi/5.f;
    pose->rKnee.angle = 0.f;
#else // punch pose for locked players
    pose = state->punchPrepPose;
    
    pose->back.angle = pi/8;
    pose->back.applicationFactor = 1.f;
    pose->neck.angle = 0.f;
    pose->neck.applicationFactor = 0.7f;
    
    pose->lShoulder.angle = -pi/5.f;
    pose->lShoulder.applicationFactor = 0.7f;
    pose->lElbow.angle = pi/4.f;
    pose->lElbow.applicationFactor = 0.4f;
    pose->rShoulder.angle = 0.1f;
    pose->rShoulder.applicationFactor = 1.f;
    pose->rElbow.angle = 7*pi/8;
    pose->rElbow.applicationFactor = 1.f;

    pose->lHip.angle = 4*pi/12.f + pi/5.f;// - pi/8.f;
    pose->lHip.applicationFactor = 0.1f;
    pose->lKnee.angle = -3*pi/8.f;
    pose->lKnee.applicationFactor = 0.1f;
    pose->rHip.angle = 0.f;//-pi/12.f + pi/5.f;
    pose->rHip.applicationFactor = 0.1f;
    pose->rKnee.angle = 0.f;
    pose->rKnee.applicationFactor = 0.1f;

    pose = state->punchExtendPose;
    
    pose->back.angle = pi/8;
    pose->back.applicationFactor = 1.f;
    pose->neck.angle = 0.f;
    pose->neck.applicationFactor = 0.7f;
    
    pose->lShoulder.angle = -pi/5.f;
    pose->lShoulder.applicationFactor = 0.7f;
    pose->lElbow.angle = pi/4.f;
    pose->lElbow.applicationFactor = 0.4f;
    pose->rShoulder.angle = pi/2.f + pi/5.f;
    pose->rShoulder.applicationFactor = 1.f;
    pose->rElbow.angle = 0.f;
    pose->rElbow.applicationFactor = 1.f;

    pose->lHip.angle = 4*pi/12.f + pi/5.f;// - pi/8.f;
    pose->lHip.applicationFactor = 0.1f;
    pose->lKnee.angle = -3*pi/8.f;
    pose->lKnee.applicationFactor = 0.1f;
    pose->rHip.angle = 0.f;//-pi/12.f + pi/5.f;
    pose->rHip.applicationFactor = 0.1f;
    pose->rKnee.angle = 0.f;
    pose->rKnee.applicationFactor = 0.1f;
#endif    
    
    // Running left pass pose
    pose = state->runningLPassPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].applicationFactor = 0.5f;
    }
    
    pose->back.angle = pi/12.f;
    pose->neck.angle = 0;
    
    pose->lShoulder.angle = 0;
    pose->lElbow.angle = pi/3.f;
    pose->rShoulder.angle = -pi/9.f;
    pose->rElbow.angle = pi/5.f;

    pose->lHip.angle = 0;
    pose->lKnee.angle = -pi/10.f;
    pose->rHip.angle = pi/12.f;
    pose->rKnee.angle = -pi/2.f;

    // Running left reach pose
    pose = state->runningLReachPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].applicationFactor = 0.5f;
    }
    
    pose->back.angle = 0;
    pose->neck.angle = 0;
    
    pose->lShoulder.angle = pi/6;
    pose->lElbow.angle = pi/2.f;
    pose->rShoulder.angle = -pi/3.f;
    pose->rElbow.angle = pi/3.f;

    pose->lHip.angle = -pi/6.f;
    pose->lKnee.angle = -pi/20.f;
    pose->rHip.angle = pi/2.f - pi/8.f;
    pose->rKnee.angle = -pi/3.f;

    *state->runningRPassPose = lrFlipPlayerPose(state->runningLPassPose);
    *state->runningRReachPose = lrFlipPlayerPose(state->runningLReachPose);

    
    // Walking left pass pose
    pose = state->walkingLPassPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].applicationFactor = 0.5f;
    }
    
    pose->back.angle = 0;
    pose->neck.angle = 0;
    
    pose->lShoulder.angle = 0;
    pose->lElbow.angle = pi/8.f;
    pose->rShoulder.angle = 0;
    pose->rElbow.angle = 0;

    pose->lHip.angle = 0;
    pose->lKnee.angle = 0;
    pose->rHip.angle = pi/12.f;
    pose->rKnee.angle = -pi/6.f;

    // Walking left reach pose
    pose = state->walkingLReachPose;
    for (int i = 0; i < playerJointCount; i++) {
        pose->joints[i].applicationFactor = 0.5f;
    }

    pose->back.angle = 0;
    pose->neck.angle = 0;
    
    pose->lShoulder.angle = pi/8;
    pose->lElbow.angle = pi/8.f;
    pose->rShoulder.angle = -pi/8.f;
    pose->rElbow.angle = pi/8.f;

    pose->lHip.angle = -pi/10.f;
    pose->lKnee.angle = -pi/20.f;
    pose->rHip.angle = pi/10.f;
    pose->rKnee.angle = 0;

    *state->walkingRPassPose = lrFlipPlayerPose(state->walkingLPassPose);
    *state->walkingRReachPose = lrFlipPlayerPose(state->walkingLReachPose);
}

internal void setPixel(GameOffscreenBuffer* buffer, int x, int y, u32 colour) {
    if (x < 0 || y < 0 || x >= (int)buffer->width || y >= (int)buffer->height) return;

     u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;  
     u8* pixel = row + buffer->bytesPerPixel * x;
     *((u32*)pixel) = colour;
}

//NOTE: This is using Bresenham's line alg.
internal void renderLine(GameOffscreenBuffer* buffer, v2 pos1, v2 pos2) {
    int x0=(int)pos1.x,x1=(int)pos2.x,y0=(int)pos1.y,y1=(int)pos2.y;
 
    int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
    int err = (dx>dy ? dx : -dy)/2, e2;
 
    for(;;){
        setPixel(buffer,x0,y0,0x00FFFFFF);
        if (x0==x1 && y0==y1) break;
        e2 = err;
        if (e2 >-dx) { err -= dy; x0 += sx; }
        if (e2 < dy) { err += dx; y0 += sy; }
    }
}

internal PoseJoint interpolateJoint(f32 t, PoseJoint* j1, PoseJoint* j2) {
    assert(j1->applicationFactor && j2->applicationFactor);

    PoseJoint result;
    f32 t1 = (1-t) * j1->applicationFactor;
    f32 t2 = t * j2->applicationFactor;
    result.applicationFactor = t1 + t2;
    result.angle = (t1 * j1->angle + t2 * j2->angle) / (t1+t2);
    return result;
}

internal PoseJoint overlayJoint(f32 t, PoseJoint* j1, PoseJoint* j2) {
    //assert(j1->applicationFactor && j2->applicationFactor);
    
    PoseJoint result;
    f32 t1 = (1-t) * j1->applicationFactor;
    f32 t2 = t * j2->applicationFactor;
    result.applicationFactor = t1 + t2;
    result.angle = (t1 * j1->angle + t2 * j2->angle) / (t1+t2);
    return result;
}

internal void updatePlayer(GameState* state, Player* pl, f32 dt, GameOffscreenBuffer* buffer) {
    if (state->playerCount == 2 && // only works for two player
        fabsf(state->players[0].playerRect->p.x -
              state->players[1].playerRect->p.x) < 300.f &&
        fabsf(state->players[0].playerRect->p.y -
              state->players[1].playerRect->p.y) < 800.f) {

        // make players face each other when close
        if (state->players[0].playerRect->p.x <
            state->players[1].playerRect->p.x) {
                
            state->players[0].direction = DIRECTION_RIGHT;
            state->players[1].direction = DIRECTION_LEFT;
        } else {
            state->players[1].direction = DIRECTION_RIGHT;
            state->players[0].direction = DIRECTION_LEFT;
        }
        //state->players[0].playerRect->colour = 0x00000000;
    } else {
        //state->players[0].playerRect->colour = 0x00FFFFFF;
        if (pl->playerRect->v.x > -700.f && pl->playerRect->accel.x > 300.f) {
            pl->direction = DIRECTION_RIGHT;    
        } else if (pl->playerRect->v.x < 700.f && pl->playerRect->accel.x < -300.f) {
            pl->direction = DIRECTION_LEFT;
        }
    }

    
    // --- MATCH POSE ---

    PlayerPose pose;
    if (pl->currentPose) { // If matching a specific pose
        PlayerPose pose = *pl->currentPose;
    } else if (pl->prevPose && pl->nextPose) {
        // interpolate pose

        if (pl->playerRect->accel.y > -800.f) // if isn't falling
            pl->strideWheelAngle -= dt*pl->playerRect->v.x / pl->strideWheelRadius;
        pl->strideWheelAngle = normAngle(pl->strideWheelAngle);

        pl->strideWheelRadius = bound(fabsf(pl->playerRect->v.x)/5.f, 75, 175);
        f32 walkRunInterpolationFactor = (pl->strideWheelRadius-75.f)/100.f;
        
        v2 strideWheelPos = pl->playerRect->p
            - V2(0, 180 - pl->strideWheelRadius);

#if 0 // debug stride wheel
        renderLine(
            buffer,
            strideWheelPos,
            strideWheelPos + rotate(
                V2(pl->strideWheelRadius,0),
                pl->strideWheelAngle));
        renderLine(
            buffer,
            strideWheelPos,
            strideWheelPos + rotate(
                V2(pl->strideWheelRadius,0),
                pl->strideWheelAngle+pi/2));
        renderLine(
            buffer,
            strideWheelPos,
            strideWheelPos + rotate(
                V2(pl->strideWheelRadius,0),
                pl->strideWheelAngle+pi));
        renderLine(
            buffer,
            strideWheelPos,
            strideWheelPos + rotate(
                V2(pl->strideWheelRadius,0),
                pl->strideWheelAngle+3*pi/2));
#endif

        PlayerPose *prevWalkPose, *prevRunPose, *nextWalkPose, *nextRunPose;
        f32 theta = pl->strideWheelAngle * (pl->direction == DIRECTION_RIGHT ? -1 : 1);
        f32 interpolationFactor;
        //1 step between 0 and pi/2
        //pass pose: 0, pi/2, pi
        //reach pose: pi/4, 3*pi/4
        if (theta < 0) theta += pi;
        if (theta <= pi/4) {
            prevRunPose = state->runningLPassPose;
            nextRunPose = state->runningLReachPose;
            prevWalkPose = state->walkingLPassPose;
            nextWalkPose = state->walkingLReachPose;
            interpolationFactor = theta / (pi/4);
        } else if (theta <= pi/2) {
            prevRunPose = state->runningLReachPose;
            nextRunPose = state->runningRPassPose;
            prevWalkPose = state->walkingLReachPose;
            nextWalkPose = state->walkingRPassPose;
            interpolationFactor = (theta-pi/4) / (pi/4);
        } else if (theta <= 3*pi/4) {
            prevRunPose = state->runningRPassPose;
            nextRunPose = state->runningRReachPose;
            prevWalkPose = state->walkingRPassPose;
            nextWalkPose = state->walkingRReachPose;
            interpolationFactor = (theta-pi/2) / (pi/4);
        } else if (theta <= pi) {
            prevRunPose = state->runningRReachPose;
            nextRunPose = state->runningLPassPose;
            prevWalkPose = state->walkingRReachPose;
            nextWalkPose = state->walkingLPassPose;
            interpolationFactor = (theta-3*pi/4) / (pi/4);
        } else assert(false);

        // fall between steps
        /*if (theta <= pi/2) {
            
        }*/

        local_persist f32 punchInterpolationFactor = 0.5f;
        punchInterpolationFactor += 0.01f;
        if (punchInterpolationFactor > 1)
            punchInterpolationFactor = 0.f;
        
        for (int i = 0; i < playerJointCount; i++) {
#if 0
            PoseJoint* prevWalkJoint = &prevWalkPose->joints[i];
            PoseJoint* nextWalkJoint = &nextWalkPose->joints[i];
            PoseJoint* prevRunJoint = &prevRunPose->joints[i];
            PoseJoint* nextRunJoint = &nextRunPose->joints[i];
            f32 prevAngle = prevWalkJoint->angle*(1-walkRunInterpolationFactor)
                + prevRunJoint->angle*walkRunInterpolationFactor;
            f32 nextAngle = nextWalkJoint->angle*(1-walkRunInterpolationFactor)
                + nextRunJoint->angle*walkRunInterpolationFactor;
            pose.joints[i].angle = prevAngle*(1-interpolationFactor)
                + nextAngle*interpolationFactor;
            pose.joints[i].applicationFactor = 1.f;
            if (pl->direction == DIRECTION_RIGHT) {
                pose.joints[i].angle *= -1;
            }
#else
            PoseJoint* prevWalkPoseJoint = &prevWalkPose->joints[i];
            PoseJoint* nextWalkPoseJoint = &nextWalkPose->joints[i];
            PoseJoint* prevRunPoseJoint = &prevRunPose->joints[i];
            PoseJoint* nextRunPoseJoint = &nextRunPose->joints[i];
            PoseJoint prevPoseJoint = interpolateJoint(
                walkRunInterpolationFactor,
                prevWalkPoseJoint, prevRunPoseJoint);
            PoseJoint nextPoseJoint = interpolateJoint(
                walkRunInterpolationFactor,
                nextWalkPoseJoint, nextRunPoseJoint);
            PoseJoint movementPoseJoint = interpolateJoint(
                interpolationFactor,
                &prevPoseJoint, &nextPoseJoint);
            PoseJoint punchPoseJoint = interpolateJoint(
                punchInterpolationFactor, &state->punchPrepPose->joints[i], &state->punchExtendPose->joints[i]);
            PoseJoint outputPoseJoint = overlayJoint(
                0.5f, &movementPoseJoint, &punchPoseJoint);
            pose.joints[i] = outputPoseJoint;
            if (pl->direction == DIRECTION_RIGHT) {
                pose.joints[i].angle *= -1;
            }
#endif
        }
    }
    
#if 1 // physics method
    for (int i = 0; i < playerJointCount; i++) {
        f32 targetAngle = pose.joints[i].angle;
        PhysicsJoint* joint = &pl->joints->joints[i];
            
        joint->targetAngle = targetAngle;

        joint->enablePID = true;
        joint->enableMotor = true;

#if 0
        if (pl->rPunching) { 
#if 0 // simple constant torque punching
            if (joint == &pl->joints->rShoulder) {
                //Get joint pos
                v2 shoulderPos = getJointPosition(&pl->joints->rShoulder);
                v2 relPos = pl->rPunchTarget - shoulderPos;

                f32 theta = atan2f(relPos.y, relPos.x);
                theta *= -1;
                theta += pl->segments->chest.angle;
                theta -= pi/2;
                joint->targetAngle = normAngle(theta);
            } else if (joint == &pl->joints->rElbow) {
                joint->targetAngle = joint->r1->angle - joint->r2->angle;
            }
#else // test punching alignment
            if (joint == &pl->joints->rShoulder) {
                v2 shoulderPos = getJointPosition(joint);
                v2 relPos = pl->rPunchTarget - shoulderPos;

                f32 theta = atan2f(relPos.y, relPos.x);
                //theta *= -1;
                //theta += pl->segments->chest.angle;
                //theta -= pi/2;
                f32 targetFistAngle = normAngle(theta);
                //NOTE: Need to make the fist fall on the
                //half-line from the shoulder along this angle
                    
                //v2 shoulderPos = getJointPosition(&pl->joints->rShoulder);

                PhysicsRect* rForearm = &pl->segments->rForearm;
                f32 forearmLength = rForearm->h/2 - rForearm->w; 
                v2 fistPos = rForearm->p +
                    rotate(V2(0, -forearmLength), rForearm->angle);
                v2 relFistPos = fistPos - shoulderPos;
                f32 curFistAngle = atan2f(relFistPos.y, relFistPos.x);

                f32 curShoulderAngle = joint->r1->angle - joint->r2->angle;
                joint->targetAngle = normAngle(curShoulderAngle - targetFistAngle + curFistAngle); 
                    
                renderPoint(buffer, fistPos, 5);
                renderPoint(buffer, shoulderPos, 5);
                renderLine(buffer, fistPos, shoulderPos);
                renderLine(buffer, shoulderPos, shoulderPos+rotate(V2(200,0), theta));
            } else if (joint == &pl->joints->rElbow) {
                PhysicsJoint* rElbow = &pl->joints->rElbow;
                f32 angleFromAlignment = getJointAngle(rElbow) - rElbow->targetAngle;
                angleFromAlignment = normAngle(angleFromAlignment);
                if (fabsf(angleFromAlignment) < pi/4 || pl->rPunchTimer > 0.f) {
                    pl->rPunchTimer += dt;
                    joint->targetAngle = (pi/4 - pl->rPunchTimer/0.2f);//fabsf(angleFromAlignment);
                    if (pl->rPunchTimer > 0.3f) pl->rPunching = false;
                    //joint->enablePID = true;
                } //else joint->targetAngle = pi;
            }
#endif
        }
#endif
    }
        
#else // static method (gives similar results)
    PhysicsJoint* j;
    j = &pl->joints->back;
    j->r1->angle = j->r2->angle + pose->backAngle;
    j = &pl->joints->neck;
    j->r1->angle = j->r2->angle + pose->neckAngle;
        
    j = &pl->joints->lShoulder;
    j->r2->angle = j->r1->angle - pose->lShoulderAngle;
    j = &pl->joints->lElbow;
    j->r2->angle = j->r1->angle - pose->lElbowAngle;
    j = &pl->joints->rShoulder;
    j->r2->angle = j->r1->angle - pose->rShoulderAngle;
    j = &pl->joints->rElbow;
    j->r2->angle = j->r1->angle - pose->rElbowAngle;

    j = &pl->joints->lHip;
    j->r2->angle = j->r1->angle - pose->lHipAngle;
    j = &pl->joints->lKnee;
    j->r2->angle = j->r1->angle - pose->lKneeAngle;
    j = &pl->joints->rHip;
    j->r2->angle = j->r1->angle - pose->rHipAngle;
    j = &pl->joints->rKnee;
    j->r2->angle = j->r1->angle - pose->rKneeAngle;

    for (int i = 0; i < playerSegmentCount; i++) {
        PhysicsRect* r = &pl->segments->segments[i];
        r->angularVel = 0;
    }
        
#endif
    
#if 0
    { // if no pose to match
        PlayerPose* pose = pl->currentPose;
        for (int i = 0; i < playerJointCount; i++) {
            PhysicsJoint* joint = &pl->joints->joints[i];
            joint->enableMotor = false;
        }
    }
#endif
    
    for (int i = 0; i < playerSegmentCount; i++) {
        PhysicsRect* rect = &pl->segments->segments[i];
        f32 angImpulse = pl->torque * dt;
        rect->angularVel += angImpulse / rect->momentOfInertia;
    }
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////// GAME UPDATE AND RENDER //////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

GAME_UPDATE_AND_RENDER(gameUpdateAndRender) {
    assert(sizeof(GameState) <= memory->permanentStorageSize);
    GameState* state = (GameState*)memory->permanentStorage;

    dt = 0.033f;
    
    if (!state->isInitialised) {
        state->randomSeed = seed;
        state->enableJoints = true;
        state->enableFriction = true;
        state->enableCollision = true;
        state->enableMotor = true;
        state->enablePIDJoints = true;
        state->enableRotationalConstraints = true;
        state->frictionCoef = 0.2f;
        state->jointPositionalBiasCoef = 0.6f;
        state->motorTargetAngVel = 2.f;
        state->maxMotorTorque = 20000000.f;
        state->backgroundColour = 0x00FFFFFF;//0x003B80FF;
        
        state->rectCount = 0;
        state->collisionIslandCount = 0;
        state->poseCount = 0;

        state->globalVel = {};
        state->globalPos = {};
        
        makeWalls(state, buffer);
        
        for (int i = 0; i < 0; i++) {
            PhysicsRect* r = &state->rects[state->rectCount++];
            CollisionIsland* ci = &state->collisionIslands[state->collisionIslandCount++];
            ci->enable = true;
            ci->count = 1;
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
            computeMassAndMomentOfInertia(r);
        }
        
        state->readyPose = &state->poses[state->poseCount++];
        state->punchPrepPose = &state->poses[state->poseCount++];
        state->punchExtendPose = &state->poses[state->poseCount++];
        state->defaultPose = &state->poses[state->poseCount++];
        state->ballPose = &state->poses[state->poseCount++];
        state->runningLPassPose = &state->poses[state->poseCount++];
        state->runningLReachPose = &state->poses[state->poseCount++];
        state->runningRPassPose = &state->poses[state->poseCount++];
        state->runningRReachPose = &state->poses[state->poseCount++];
        state->walkingLPassPose = &state->poses[state->poseCount++];
        state->walkingLReachPose = &state->poses[state->poseCount++];
        state->walkingRPassPose = &state->poses[state->poseCount++];
        state->walkingRReachPose = &state->poses[state->poseCount++];

        state->playerCount = 2;
        makePlayer(state, &state->players[0], buffer);
        makePlayer(state, &state->players[1], buffer);
        makePlayerPoses(state);
        
        state->isInitialised = true;
    }

    //NOTE: TEMP
    makePlayerPoses(state);

    
    // ---- INPUT ----
    
#if 0
    for (int i = 0; i < state->playerCount; i++) {
        state->players[i].playerRect->v.x +=
            dt*2400.f*input->controllers[i+1].lStick.endX;
        if (wasTapped(input->controllers[i+1].yButton))
            state->players[i].playerRect->v.y += 1000.f;
    }
#else // debug single controller input
    GameControllerInput* controller = &input->controllers[1];
    state->players[0].playerRect->v.x += dt*2400.f*controller->lStick.endX;
    if (wasTapped(controller->yButton))
        state->players[0].playerRect->v.y += 1000.f;
        
    state->players[1].playerRect->v.x += dt*2400.f*controller->rStick.endX;
    if (wasTapped(controller->aButton))
        state->players[1].playerRect->v.y += 1000.f;
#endif
    
    
    // ---- UPDATE AND RENDER ----

    for (int i = 0; i < state->rectCount; i++) {
        PhysicsRect* r = &state->rects[i];
        r->accel = (r->v - r->lastV) / dt;
        r->lastV = r->v;
    }
    
    for (int j = 0; j < state->playerCount; j++) {
        for (int i = 0; i < 10; i++) {
            state->players[j].joints->joints[i].targetAngVel
                = state->motorTargetAngVel;
        }
    }
    
    state->globalForce = {};
    state->globalAccel = V2(0,-1500.f);

    //state->playerBody.torque = keyboardTorque;
    
    state->globalVel = state->globalAccel/5.f*(f32)state->globalAccelTimer/100.f;
    state->globalPos += dt*state->globalVel;

    renderBackground(state, buffer);

    
#if 1 // test render so we can see debug overlays
    // --- RENDER ---
    for (int i = 0; i < state->rectCount; i++) {
        PhysicsRect* r = &state->rects[i];
        renderRectangle(buffer, r, r->colour);
    }
    for (int i = 0; i < state->playerCount; i++) { // Render player to get rects in correct order
        PhysicsRect* r;
        PlayerSegments* segs = state->players[i].segments;
        
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
        updatePlayer(state, &state->players[i], dt, buffer);
    
    // --- SIMULATE ---
        
    int iterCount = 30;
    f32 h = dt/iterCount;
    for (int iter = 0; iter < iterCount; iter++) {
        // --- UPDATE POSTION ---
        for (int i = 0; i < state->rectCount; i++) {
            PhysicsRect* r = &state->rects[i];
            
            if (r->fixed) {
                r->v = V2(0,0);
                r->angularVel = 0;
                r->mass = FLT_MAX;
                r->momentOfInertia = FLT_MAX;
            } else {
                r->v += h*state->globalForce/r->mass;
                r->v += h*state->globalAccel;
            }
            
            r->p += h * r->v;
            r->angle += h * r->angularVel;
            r->angle = normAngle(r->angle);
        }

        for (int i = 0; i < state->playerCount; i++) { // fix player abdomen to playerRect
            state->players[i].playerRect->angle = 0;
            state->players[i].playerRect->angularVel = 0;
            
            PhysicsRect* r = &state->players[i].segments->abdomen;
            r->p = state->players[i].playerRect->p;
            r->v = state->players[i].playerRect->v;
            //f32 newAngle = 0.15f;
            f32 newAngle = state->players[i].playerRect->angle -
                bound(state->players[i].playerRect->accel.x/20000.f,-pi/6,pi/6);
            r->angle += dt*bound((newAngle - r->angle)/dt,-0.2f,0.2f);
            r->angularVel = state->players[i].playerRect->angularVel;
        }

        
        // --- COLLIDE ---
        if (state->enableCollision) {
            state->collisionManifoldCount = 0;
            
            for (int i1 = 0; i1 < state->collisionIslandCount; i1++) {
                CollisionIsland* ci1 = &state->collisionIslands[i1];
                if (!ci1->enable) continue;
                for (int i2 = 0; i2 < i1; i2++) {
                    CollisionIsland* ci2 = &state->collisionIslands[i2];
                    if (!ci2->enable) continue;
                    for (int ri1 = 0; ri1 < ci1->count; ri1++) {
                        PhysicsRect* r1 = &ci1->rects[ri1];
                        Polygon p1; computeRectVertices(r1,&p1);
                        for (int ri2 = 0; ri2 < ci2->count; ri2++) {
                            PhysicsRect* r2 = &ci2->rects[ri2];
                            Polygon p2; computeRectVertices(r2, &p2);
                            if (r1->fixed && r2->fixed) continue;
                            CollisionManifold collisionManifold;
                            if (doPolygonsIntersect(&p1,&p2,&collisionManifold)) {
                                assert(state->collisionManifoldCount < maxCollsionManifolds);
                                collisionManifold.r1 = r1;
                                collisionManifold.r2 = r2;
                                state->collisionManifolds[state->collisionManifoldCount++]
                                    = collisionManifold;
                            }
                        }
                    }
                }
            }
            
            for (int i = 0; i < state->collisionManifoldCount; i++) {
                resolveCollision(state,h,&state->collisionManifolds[i]);
            }
        }

        // --- RESOLVE JOINT CONSTRAINTS ---
        for (int i = 0; i < state->jointCount; i++) {
            resolveJointConstraint(state, &state->joints[i], h);
        }
    }

#if 0 // actual render position, just need to move this for a test
    // --- RENDER ---
    for (int i = 0; i < state->rectCount; i++) {
        PhysicsRect* r = &state->rects[i];
        renderRectangle(buffer, r, r->colour);
    }
#endif
}
