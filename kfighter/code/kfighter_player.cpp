/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

#include "kfighter.h"
#include "kfighter_util.h"

internal void makePlayer(GameState* state, Player* pl, GameOffscreenBuffer* buffer) {
    CollisionIsland* ci = &state->collisionIslands[state->collisionIslandCount++];
    ci->enable = true;
    ci->count = playerSegmentCount;
    ci->rects = &state->rects[state->rectCount];
    
    pl->segments = (PlayerSegments*)ci->rects;
    PhysicsRect* r;
    f32 margin = 50;
    f32 pX = rand(state)*(buffer->width-2*margin) + margin;
    f32 pY = rand(state)*(buffer->height-2*margin) + margin;
    for (int i = 0; i < ci->count; i++) {
        r = &state->rects[state->rectCount++];
        r->fixed = false;
        r->enableFriction = true;
        r->p.x = pX + (rand(state)-.5f)*50;
        r->p.x = pY + (rand(state)-.5f)*50;
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
    state->jointCount += playerJointCount;
    
#define MAKE_JOINT(joint, part1, part2, min, max, vOffsetFactor) \
    {                                                            \
        PhysicsJoint* j = &pl->joints->joint;                    \
        j->r1 = &seg->part1;                                     \
        j->r2 = &seg->part2;                                     \
        j->relPos1 = -V2(                                        \
            0,                                                   \
            j->r1->h/2*(vOffsetFactor) - j->r1->w/2);            \
        j->relPos2 = +V2(0, j->r2->h/2 - j->r2->w/2);            \
        j->minTheta = (min)*pi;                                  \
        j->maxTheta = (max)*pi;                                  \
        j->targetAngVel = 0;                                     \
        j->targetAngle = 0;                                      \
        j->pidIntegralTerm = 0;                                  \
        j->enableMotor = false;                                  \
        j->enablePID = false;                                    \
        j->enable = true;                                        \
    }

#if 0 // Debug constraints
    MAKE_JOINT(neck,      head,    chest,    -0.30f, 0.40f, 5.f/3.f);
    MAKE_JOINT(back,      chest,   abdomen,  -0.00f, 0.00f, 1.f);
    MAKE_JOINT(lShoulder, chest,   lBicep,   -0.70f, 0.90f, -0.0f);
    MAKE_JOINT(lElbow,    lBicep,  lForearm, -0.00f, 0.80f, 1.f);
    MAKE_JOINT(rShoulder, chest,   rBicep,   -0.70f, 0.90f, -0.0f);
    MAKE_JOINT(rElbow,    rBicep,  rForearm, -0.00f, 0.80f, 1.f);
    MAKE_JOINT(lHip,      abdomen, lThigh,   -0.30f, 0.60f, 1.f);
    MAKE_JOINT(lKnee,     lThigh,  lShin,    -0.70f, 0.00f, 1.f);
    MAKE_JOINT(rHip,      abdomen, rThigh,   -0.30f, 0.60f, 1.f);
    MAKE_JOINT(rKnee,     rThigh,  rShin,    -0.70f, 0.00f, 1.f);
#else
    MAKE_JOINT(neck,      head,    chest,    -0.30f, 0.40f, 5.f/3.f);
    MAKE_JOINT(back,      chest,   abdomen,  -0.05f, 0.20f, 1.f);
    MAKE_JOINT(lShoulder, chest,   lBicep,   -0.70f, 0.90f, -0.0f);
    MAKE_JOINT(lElbow,    lBicep,  lForearm, -0.00f, 0.80f, 1.f);
    MAKE_JOINT(rShoulder, chest,   rBicep,   -0.70f, 0.90f, -0.0f);
    MAKE_JOINT(rElbow,    rBicep,  rForearm, -0.00f, 0.80f, 1.f);
    MAKE_JOINT(lHip,      abdomen, lThigh,   -0.30f, 0.60f, 1.f);
    MAKE_JOINT(lKnee,     lThigh,  lShin,    -0.70f, 0.00f, 1.f);
    MAKE_JOINT(rHip,      abdomen, rThigh,   -0.30f, 0.60f, 1.f);
    MAKE_JOINT(rKnee,     rThigh,  rShin,    -0.70f, 0.00f, 1.f);
#endif
    
#undef MAKE_JOINT

    // --- ADDITIONAL VARIABLES ---
    s32 poseID = randIntBetween(state, -1, state->poseCount);
    pl->currentPose = ~poseID ? state->poses + poseID : 0;
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
    // Match pose
    if (pl->currentPose) { // If matching a specific pose
        PlayerPose pose = *pl->currentPose;
        for (int i = 0; i < playerJointCount; i++) {
            f32 targetAngle = pose.joints[i].angle;
            PhysicsJoint* joint = &pl->joints->joints[i];
            
            joint->targetAngle = targetAngle;

            joint->enablePID = true;
            joint->enableMotor = true;
        }
    } else {
        for (int i = 0; i < playerJointCount; i++) {
            PhysicsJoint* joint = &pl->joints->joints[i];
            joint->enablePID = false;
            joint->enableMotor = false;
        }
    }
}
