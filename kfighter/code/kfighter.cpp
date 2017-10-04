/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

/* TODO: Lots to do before I have a game
   Physics:
    - Add collsion manifolds 
    - Apply collsions with other constraints
    - Joint friction
    - Static friction
    - Torque constraints
    - Collision boucing (bias)
    - Collisoin islands
    - Capsule collision detection
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
    - Controller input
   Refactor:
    - Bring out control variables into the state
    - Make the seed come from the environment
    - Bring common routines into functions
    - Group like code
    - Remove dead #if 0's
    - Move other #if 0's to own fuction
    - Memory arenas
    - Bring physics out into its own file
 */

#include "kfighter.h"
#define rand __rand
#include <math.h>
#include <float.h>
#undef rand

inline f32 sqr(f32 x) {return x*x;}

inline v2 V2(f32 x, f32 y) {
    v2 result;
    result.x = x;
    result.y = y;
    return result;
}

inline v2 operator+(v2 vec) {return vec;}
inline v2 operator-(v2 vec) {
    v2 result;
    result.x = -vec.x;
    result.y = -vec.y;
    return result;
}
inline v2 operator+(v2 lhs, v2 rhs) {
    v2 result;
    result.x = lhs.x + rhs.x;
    result.y = lhs.y + rhs.y;
    return result;
}
inline v2 operator-(v2 lhs, v2 rhs) {return lhs+(-rhs);}
inline v2 operator*(f32 scalar, v2 vec) {
    v2 result;
    result.x = scalar * vec.x;
    result.y = scalar * vec.y;
    return result;
}
inline v2 operator*(v2 vec, f32 scalar) {return scalar*vec;}
inline v2 operator/(v2 vec, f32 scalar) {
    if (scalar == 0) return vec;
    else return (1/scalar)*vec;
}

inline v2 &operator+=(v2 &lhs, v2 rhs) {return lhs = lhs+rhs;}
inline v2 &operator-=(v2 &lhs, v2 rhs) {return lhs = lhs-rhs;}
inline v2 &operator*=(v2 &lhs, f32 scalar) {return lhs = lhs*scalar;}
inline v2 &operator/=(v2 &lhs, f32 scalar) {
    if (scalar == 0) return lhs;
    else return lhs = lhs/scalar;
}

inline bool operator==(v2 lhs, v2 rhs) {return mag(lhs-rhs) < epsilon;}
inline bool operator!=(v2 lhs, v2 rhs) {return !(lhs == rhs);}

inline f32 sqrmag(v2 vec) {return sqr(vec.x) + sqr(vec.y);}
inline f32 mag(v2 vec) {return sqrtf(sqrmag(vec));}
inline v2 norm(v2 vec) {
    f32 magnitude = mag(vec);
    if (magnitude == 0)
        return V2(1,0);
    return vec/mag(vec);
}
inline v2 perp(v2 vec) {return V2(-vec.y,vec.x);}
inline f32 dot(v2 vec1, v2 vec2) {
    f32 p1 = vec1.x*vec2.x;
    f32 p2 = vec1.y*vec2.y;
    f32 result = p1+p2;
    return result;
}
inline f32 cross(v2 vec1, v2 vec2) {
    return vec1.x*vec2.y - vec1.y*vec2.x;
}
inline v2 rotate(v2 vec, f32 angle) {
    v2 result;
    result.x = cosf(angle)*vec.x - sinf(angle)*vec.y;
    result.y = sinf(angle)*vec.x + cosf(angle)*vec.y;
    return result;
}

//NOTE: Computes vertices anticlockwise
internal void computeRectVertices(PhysicsRect* r, Polygon* res) {
    res->count = 4;
    res->center = r->p;
    assert(res->count <= maxVerticesInPolygon);
    res->verts[0] = V2(r->w/2.f,r->h/2.f);
    res->verts[1] = V2(-r->w/2.f,r->h/2.f);
    res->verts[2] = -res->verts[0];
    res->verts[3] = -res->verts[1];
    for (int i=0;i<res->count;i++)
        res->verts[i] = rotate(res->verts[i],r->angle);
    for (int i=0;i<res->count;i++)
        res->verts[i] += r->p;
}

#if 0
//NOTE: There is an O(n) implementation but for rects it doesn't make too much difference
//NOTE: This funciton is destructive
internal void computeMinkowskiDifference(Polygon* p1, Polygon* p2, Polygon* res) {
    int c1=p1->count, c2=p2->count;
    v2 tmpVerts[maxVerticesInPolygon*maxVerticesInPolygon];

    // Ensure that the first element is the right-most
    int iMaxXP1=0, iMaxXP2=0;
    for (int i = 0; i < c1; i++)
        if (p1->verts[i].x > p1->verts[iMaxXP1])
            iMaxXP1 = i;
    for (int i = 0; i < c1; i++)
        tmpVerts[i] = p1->verts[(i+iMaxXP1)%c1];
    for (int i = 0; i < c1; i++)
        p1->verts[i] = tmpVerts[i];
    for (int i = 0; i < c2; i++)
        if (p2->verts[i].x > p2->verts[iMaxXP2])
            iMaxXP2 = i;
    for (int i = 0; i < c2; i++)
        tmpVerts[i] = p2->verts[(i+iMaxXP2)%c2];
    for (int i = 0; i < c2; i++)
        p1->verts[i] = tmpVerts[i];

    int i=0, j=0;
    while ()
    
#if 0
        // Compute directions pointing inwards
        v2 dirs1[maxVerticesInPolygon];
    for (int i = 0; i < c1; i++)
        dirs1[i] = perp(p1->verts[(i+1)%c1] - p1->verts[i]);
    
    v2 dirs2[maxVerticesInPolygon];
    for (int i = 0; i < c2; i++)
        dirs2[i] = perp(p1->verts[(i+1)%c2] - p1->verts[i]);
#endif
    
    for (int i1 = 0; i1 < c1; i1++) {
        for (int i2 = 0; i2 < c2; i2++) {
            //tmpVerts[]
        }
    }
}
#endif

struct SupportVectors {
    int count;
    int indices[2];
};

/*internal IndexList supportVectors(Polygon* p, v2 vec) {
  f32 maxProjected = -FLT_MAX;
  f32 maxProjected2 = -FLT_MAX;
  int maxIndex;
  for (int i = 0; i < p->count; i++) {
  f32 projected = dot(vec, p->verts[i]);
  if (projected > maxProjected) {
  maxProjected = projected;
  maxIndex = i;
  }
  }
  return maxIndex;
  }*/

internal int supportVector(Polygon* p, v2 vec) {
    f32 maxProjected = -FLT_MAX;
    int maxIndex;
    for (int i = 0; i < p->count; i++) {
        f32 projected = dot(vec, p->verts[i]);
        if (projected > maxProjected) {
            maxProjected = projected;
            maxIndex = i;
        }
    }
    return maxIndex;
}

internal bool doPolygonsIntersect(Polygon* a, Polygon* b, CollisionInfo* info) {
    f32 minOverlap = FLT_MAX;
    Polygon* minShape;
    Polygon* vertexShape;
    
    for (int shape = 0; shape < 2; shape++) {
        Polygon* p = (shape == 0 ? a : b);
        for (int i1 = 0; i1 < p->count; i1++) {
            int i2 = (i1+1) % p->count;
            v2 p1 = p->verts[i1];
            v2 p2 = p->verts[i2];
            v2 normal = perp(p1-p2); //pointing outward

            f32 maxA = -FLT_MAX;
            f32 minA = FLT_MAX;
            for (int i = 0; i < a->count; i++) {
                f32 projected = dot(normal, a->verts[i]);
                minA = min(minA, projected);
                maxA = max(maxA, projected);
            }
            
            f32 maxB = -FLT_MAX;
            f32 minB = FLT_MAX;
            for (int i = 0; i < b->count; i++) {
                f32 projected = dot(normal, b->verts[i]);
                minB = min(minB, projected);
                maxB = max(maxB, projected);
            }
            
            if (maxA < minB || maxB < minA) return false;

            f32 overlap = min(maxA-minB,maxB-minA) / mag(normal);
            if (overlap < minOverlap) {
                info->normal = normal;
                minOverlap = overlap;
                vertexShape = (p == a ? b : a);
                minShape = (maxA-minB > maxB-minA ? a : b);
            }
        }
    }

    f32 sign = (vertexShape == minShape ? -1.f : 1.f);
    info->normal *= sign;
    int supportIndex;
    if (vertexShape == a || vertexShape == b)
        supportIndex = supportVector(vertexShape, info->normal);
    else int x=5;
    v2 supportVertex = vertexShape->verts[supportIndex];
    info->depth = minOverlap;
    info->pos = supportVertex;// + sign/2.f * minOverlap * info->normal / sqrmag(info->normal);
    //NOTE: Normal is always pointing from a to b
    info->normal = (vertexShape == a ? -1.f : 1.f) * norm(info->normal);
    return true;
}

internal void renderWeirdGradient(GameOffscreenBuffer* buffer, s32 offsetX, s32 offsetY) {
    for (u32 y = 0; y < buffer->height; y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (u32 x = 0; x < buffer->width; x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            pixel[0] = (x+offsetX) & 255; // Blue
            pixel[1] = (y+offsetY) & 255; // Green
            pixel[2] = 0; //(x ^ y) & 255; // Red
            pixel[3] = 0; // Unused
        }
    }
}

internal void renderPlayer(GameOffscreenBuffer* buffer, int xPos, int yPos, f32 size) {
    int halfPlayerSize = (int)(20*(0.8f*sqrt(sqrt(size))+0.2f));
    for (int y = max(yPos-halfPlayerSize,0); y < min(yPos+halfPlayerSize, (int)buffer->height); y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (int x = max(xPos-halfPlayerSize,0); x < min(xPos+halfPlayerSize, (int)buffer->width); x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            pixel[0] = 0; // Blue
            pixel[1] = 0; // Green
            pixel[2] = 255; // Red
            pixel[3] = 0; // Unused
        }
    }
}

internal void renderSquare(GameOffscreenBuffer* buffer, int xPos, int yPos, int size) {
    int halfSize = size/2;
    for (int y = max(yPos-halfSize,0); y < min(yPos+halfSize, (int)buffer->height); y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (int x = max(xPos-halfSize,0); x < min(xPos+halfSize, (int)buffer->width); x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            pixel[0] = 255; // Blue
            pixel[1] = 255; // Green
            pixel[2] = 255; // Red
            pixel[3] = 0; // Unused
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

//TODO: Is this random enough?
u32 randomSeed = 2;

internal void seedRandomNumberGenerator() {
    randomSeed = (u32)(1103515245 * randomSeed + 12345);
}

//NOTE: Generates between 0 and 1
internal f32 rand() {
    seedRandomNumberGenerator();
    return randomSeed / (float)(0x100000000LL);
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
    return wasReleased(button) || button.halfTransitionCount >= 2;
}

internal void applyImpulsePair(
    GameState* state, f32 dt,
    v2 normal,
    f32 impulse,
    v2 pos,
    PhysicsRect* r1,
    PhysicsRect* r2) {
    
    normal = norm(normal);
    v2 rel1 = pos - r1->p;
    v2 rel2 = pos - r2->p;
    v2 vel1 = r1->v + r1->angularVel * perp(rel1);
    v2 vel2 = r2->v + r2->angularVel * perp(rel2);

    if (!r1->fixed) {
        r1->v += impulse * normal / r1->mass ;
        r1->angularVel += impulse * cross(rel1, normal) / r1->momentOfInertia;
    }
    if (!r2->fixed) {
        r2->v += impulse * (-normal) / r2->mass;
        r2->angularVel += impulse * cross(rel2, -normal) / r2->momentOfInertia;
    } 
}

internal void resolveCollision(
    GameState* state, f32 dt,
    CollisionInfo collisionInfo,
    PhysicsRect* r1,
    PhysicsRect* r2) {

    v2 pos = collisionInfo.pos;
    v2 normal = collisionInfo.normal;
    f32 depth = collisionInfo.depth;
    normal = norm(normal);
    v2 rel1 = pos - r1->p;
    v2 rel2 = pos - r2->p;
    v2 vel1 = r1->v + r1->angularVel * perp(rel1);
    v2 vel2 = r2->v + r2->angularVel * perp(rel2);

    bool fixed = (r1->fixed || r2->fixed);
    if (!r1->fixed) r1->p += depth*normal / (fixed ? 1.f : 2.f);
    if (!r2->fixed) r2->p -= depth*normal / (fixed ? 1.f : 2.f); 

    //v2 frictionImpulse = dot(perp(normal),vel1-vel2)*perp(normal);
    //if (r1->fixed) frictionImpulse *= r2->mass;
    //ese if (r2->fixed) frictionImpulse *= r1->mass;
    //else frictionImpulse *= r1->mass*r2->mass / (r1->mass + r2->mass);
    //frictionImpulse *= 0.1f;
#if 0
    // If they are moving toward each other
    if (dot(vel1-vel2, normal) < 0) {
        v2 impulse;

        if (r1->fixed) impulse = vel2*r2->mass;
        else if (r2->fixed) impulse = -vel1*r1->mass;
        else impulse = (vel2*r1->mass*r2->mass - vel1*r2->mass*r1->mass) / (r1->mass + r2->mass);
        impulse = dot(impulse, normal)*normal;
        impulse *= 0.5f;
        //impulse -= frictionImpulse;
        if (!r1->fixed) {
            r1->v += impulse / r1->mass;
            f32 angularImpulse1 = dot(perp(rel1), impulse);
            r1->angularVel += angularImpulse1 / r1->momentOfInertia;
            if (fabsf(r1->angularVel) > 2.f)
                state->debugPause = true;
        }
        if (!r2->fixed) {
            r2->v -= impulse / r2->mass;
            f32 angularImpulse2 = dot(perp(rel2), impulse);
            r2->angularVel -= angularImpulse2 / r2->momentOfInertia;
            if (fabsf(r2->angularVel) > 2.f)
                state->debugPause = true;
        }
    }
#else
    PhysicsRect r1Before = *r1;
    PhysicsRect r2Before = *r2;

    f32 angularMomentum1Before = r1->angularVel * r1->momentOfInertia;
    f32 angularMomentum2Before = r2->angularVel * r2->momentOfInertia;
    f32 angularMomentumBefore = angularMomentum1Before + angularMomentum2Before;
    
    f32 linEnergy1Before = 0.5f * r1->mass * sqrmag(r1->v);
    f32 rotEnergy1Before =
        0.5f * r1->momentOfInertia * sqr(r1->angularVel);
    f32 kineticEnergy1Before = linEnergy1Before + rotEnergy1Before;
    f32 potentialEnergy1Before = -r1->mass * dot(r1->p, state->globalAccel);
    f32 energy1Before = kineticEnergy1Before + potentialEnergy1Before;
    
    f32 linEnergy2Before = 0.5f * r2->mass * sqrmag(r2->v);
    f32 rotEnergy2Before =
        0.5f * r2->momentOfInertia * sqr(r2->angularVel);
    f32 kineticEnergy2Before = linEnergy2Before + rotEnergy2Before;
    f32 potentialEnergy2Before = -r2->mass * dot(r2->p, state->globalAccel);
    f32 energy2Before = kineticEnergy2Before + potentialEnergy2Before;
    
    f32 kineticEnergyBefore = kineticEnergy1Before + kineticEnergy2Before;
    //f32 potentialEnergyBefore = potentialEnergy1Before + potentialEnergy2Before;
    
    f32 totalEnergyBefore = kineticEnergyBefore;//energy1Before + energy1Before;

    f32 moment1 = (r1->fixed) ? 0 : sqr(cross(rel1, normal))/r1->momentOfInertia;
    f32 moment2 = (r2->fixed) ? 0 : sqr(cross(rel2, -normal))/r2->momentOfInertia;
    f32 momentSum = moment1 + moment2;
    f32 invMassSum = 0;
    if (!r1->fixed) invMassSum += 1.f/r1->mass;
    if (!r2->fixed) invMassSum += 1.f/r2->mass;
    f32 k = -1.f/(momentSum+invMassSum);
    f32 linVel1 = dot(normal, r1->v + dt * state->globalAccel);
    f32 linVel2 = dot(-normal, r2->v + dt * state->globalAccel);
    f32 angVel1 = cross(rel1, normal)*r1->angularVel;
    f32 angVel2 = cross(rel2, -normal)*r2->angularVel;
    f32 impulse = k * (linVel1 + linVel2 + angVel1 + angVel2);

    //impulse *= 0.4f;
    if (impulse >= 0) applyImpulsePair(state, dt, normal, impulse, pos, r1, r2);
    bool friction = true;
    if (friction) {
        v2 tangent = perp(normal);
        f32 moment1 = (r1->fixed) ? 0 : sqr(cross(rel1, tangent))/r1->momentOfInertia;
        f32 moment2 = (r2->fixed) ? 0 : sqr(cross(rel2, -tangent))/r2->momentOfInertia;
        f32 momentSum = moment1 + moment2;
        f32 invMassSum = 0;
        if (!r1->fixed) invMassSum += 1.f/r1->mass;
        if (!r2->fixed) invMassSum += 1.f/r2->mass;
        f32 k = -1.f/(momentSum+invMassSum);
        f32 linVel1 = dot(tangent, r1->v + dt * state->globalAccel);
        f32 linVel2 = dot(-tangent, r2->v + dt * state->globalAccel);
        f32 angVel1 = cross(rel1, tangent)*r1->angularVel;
        f32 angVel2 = cross(rel2, -tangent)*r2->angularVel;
        f32 impulse = k * (linVel1 + linVel2 + angVel1 + angVel2);
        applyImpulsePair(state, dt, tangent, 0.2f*impulse, pos, r1, r2);
    }

    f32 angularMomentum1After = r1->angularVel * r1->momentOfInertia;
    f32 angularMomentum2After = r2->angularVel * r2->momentOfInertia;
    f32 angularMomentumAfter = angularMomentum1After + angularMomentum2After;
    
    f32 linEnergy1After = 0.5f * r1->mass * sqrmag(r1->v);
    f32 rotEnergy1After =
        0.5f * r1->momentOfInertia * sqr(r1->angularVel);
    f32 energy1After = linEnergy1After + rotEnergy1After;

    f32 linEnergy2After = 0.5f * r2->mass * sqrmag(r2->v);
    f32 rotEnergy2After =
        0.5f * r2->momentOfInertia * sqr(r2->angularVel);
    f32 energy2After = linEnergy2After + rotEnergy2After;
    
    f32 totalEnergyAfter = energy1After + energy2After;
    if (fabsf(totalEnergyBefore - totalEnergyAfter) > epsilon) 
        state->debugPause = true;                                          
#endif
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////// GAME UPDATE AND RENDER //////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

GAME_UPDATE_AND_RENDER(gameUpdateAndRender) {
    assert(sizeof(GameState) <= memory->permanentStorageSize);
    GameState* state = (GameState*)memory->permanentStorage;

//    if (state->debugPause) return;
    dt = 0.033f;
    bool constraints = true;
    bool debugInverseRects = false;
    bool awesomePhysics = false;
    bool linkToTop = true; // Whether the string is connnected to the top of the screen
    bool centerJoints = false; // always put joints in center for debugging
    
    if (!state->isInitialised) {
        state->fixedRectCount = 4;
        state->stringSegmentRectCount = 4;
        state->playerSegmentRectCount = 11;
        state->freeRectCount = 5;
        state->stringEnable = false;
        state->stringJointCount = state->stringSegmentRectCount - 1 + (int)linkToTop;
        state->playerJointCount = 10;
        
        state->offsetX = 0;
        state->offsetY = 0;

        state->playerX = 100;
        state->playerY = 100;

        state->particleCount = 0;
        for (int i = 0; i < state->particleCount; i++) {
            state->particles[i].p.x = buffer->width/2.f;
            state->particles[i].p.y = buffer->height/2.f;
            state->particles[i].v.x = (rand()-0.5f)*2000.f;
            state->particles[i].v.y = (rand())*1000.f;
            state->particles[i].mass = 10.f;
            state->globalAccelTimer = 0;
        }

        state->playerCount = 0;
        for (int i = 0; i < state->playerCount; i++) {
            PhysicsPlayer* pl = &state->players[i];
            pl->w = 100.f;
            pl->h = 100.f;
            pl->p.x = rand()*buffer->width;
            pl->p.y = rand()*buffer->height;
            pl->v.x = (rand()-0.5f)*500.f;
            pl->v.y = (rand())*1000.f;
            pl->angle = (rand()*2*pi);
            pl->angularVel = (rand()*2*pi);
            pl->mass = 100.f;
            pl->momentOfInertia = pl->mass * 
                (sqr(pl->w)+sqr(pl->h))/12;
        }
        
        state->rectCount =
            state->fixedRectCount +
            state->stringSegmentRectCount +
            state->playerSegmentRectCount +
            state->freeRectCount;
        for (int i = 0; i < state->rectCount; i++) {
            PhysicsRect* r = &state->rects[debugInverseRects ? state->rectCount-1-i : i];
            r->w = rand()*300.f+50.f;
            r->h = rand()*100.f+20.f;
            r->p.x = rand()*buffer->width;
            r->p.y = rand()*buffer->height;
            r->v.x = (rand()-0.5f)*20.f;
            r->v.y = (rand()-0.5f)*20.f;
            r->angle = (rand()*2*pi);
            r->angularVel = (rand()*pi/4.f);
            r->colour = 0x00FFFFFF;
            r->fixed = (i < state->fixedRectCount);
            if (r->fixed) {
                r->v = V2(0,0);
                r->angularVel = 0;
                r->mass = FLT_MAX;
                r->momentOfInertia = FLT_MAX;
            }
            if (i == 0 && r->fixed) {
                r->w = (f32)buffer->width;
                r->h = 50;
                r->p.x = (f32)buffer->width/2.f;
                r->p.y = -r->h/2;
                r->angle = 0;
            } else if (i == 1 && r->fixed) {
                r->w = (f32)buffer->width;
                r->h = 50;
                r->p.x = (f32)buffer->width/2.f;
                r->p.y = (f32)buffer->height+r->h/2;
                r->angle = 0;
            } else if (i == 2 && r->fixed) {
                r->w = 50;
                r->h = (f32)buffer->height;
                r->p.x = 0-r->w/2;
                r->p.y = (f32)buffer->height/2.f;
                r->angle = 0;
            } else if (i == 3 && r->fixed) {
                r->w = 50;
                r->h = (f32)buffer->height;
                r->p.x = (f32)buffer->width+r->w/2;
                r->p.y = (f32)buffer->height/2.f;
                r->angle = 0;
            } else if (i < state->fixedRectCount + state->stringSegmentRectCount && constraints) { // test limbs
                r->w = state->stringSegmentRectCount > 8 ? 10.f : 20.f;
                r->h = 300.f/state->stringSegmentRectCount + r->w;
                r->p.x = (f32)buffer->width/2.f;
                r->p.y = (f32)buffer->height - r->h/2 + r->w/2 - (r->h - r->w)*(i-4);
                r->angle = 0;
                //r->angularVel = 0;
                //r->v = V2(0,0);
            } else if (i < state->fixedRectCount
                       + state->stringSegmentRectCount
                       + state->playerSegmentRectCount) {
                int j = i - state->fixedRectCount - state->stringSegmentRectCount;
                r->colour = 0x00FF0000;
                
                if (j == 0) { //head
                    r->w = 60.f;
                    r->h = 60.f;
                    state->playerBody.segments = (PlayerSegments*)r;
                } else { //all others
                    r->w = 20.f;
                    r->h = 90.f;

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
            }
            
            r->mass = r->w*r->h*0.01f;
            r->momentOfInertia = r->mass *
                (sqr(r->w)+sqr(r->h))/12.f;
        }

        bool ropeSwing = true;
        state->jointCount = constraints ?
            state->stringJointCount + state->playerJointCount + (int)ropeSwing : 0;
        for (int i = 0; i < state->jointCount; i++) {
            PhysicsJoint* j = &state->joints[i];
            j->enable = true;

            if (i == 0 && linkToTop) { //string to ceiling joint
                j->r1 = &state->rects[1]; //ceiling
                j->r2 = &state->rects[4];
                j->relPos1 = V2(0, -j->r1->h/2);
                j->relPos2 = V2(0, j->r2->h/2 - j->r2->w/2);
                j->minTheta = -pi/4;
                j->maxTheta = pi/4;
            } else if (i < state->stringJointCount) { //internal string joints
                j->r1 = &state->rects[i+4-(int)linkToTop];
                j->r2 = &state->rects[i+5-(int)linkToTop];
                j->relPos1 = centerJoints ? V2(0,0) :
                    -V2(0, j->r1->h/2 - j->r1->w/2);
                j->relPos2 = centerJoints ? V2(0,0) :
                    +V2(0, j->r2->h/2 - j->r2->w/2);
                j->minTheta = -pi/18;
                j->maxTheta = pi/18;
            } else if (i < state->stringJointCount + state->playerJointCount) {
                int k = i - state->stringJointCount;
                PlayerSegments* seg = state->playerBody.segments;
#define MAKE_JOINT(x, part1, part2, min, max, vOffsetFactor)       \
                if (k == x) {                                      \
                    j->r1 = &seg->part1;                           \
                    j->r2 = &seg->part2;                           \
                    j->relPos1 = -V2(                              \
                        0,                                         \
                        j->r1->h/2*(vOffsetFactor) - j->r1->w/2);  \
                    j->relPos2 = +V2(0, j->r2->h/2 - j->r2->w/2);  \
                    j->minTheta = (min)*pi;                        \
                    j->maxTheta = (max)*pi;                        \
                }
                
#if 1 // This configuration works but doesn't seem to give good results
                MAKE_JOINT(0, head,    chest,    -0.30f, 0.40f, 5.f/3.f);
                MAKE_JOINT(1, chest,   abdomin,  -0.05f, 0.20f, 1);
                MAKE_JOINT(2, chest,   lBicep,   -0.70f, 0.90f, -0.0f);
                MAKE_JOINT(3, lBicep,  lForearm, -0.00f, 0.80f, 1);
                MAKE_JOINT(4, chest,   rBicep,   -0.70f, 0.90f, -0.0f);
                MAKE_JOINT(5, rBicep,  rForearm, -0.00f, 0.80f, 1);
                MAKE_JOINT(6, abdomin, lThigh,   -0.30f, 0.60f, 1);
                MAKE_JOINT(7, lThigh,  lShin,    -0.70f, 0.00f, 1);
                MAKE_JOINT(8, abdomin, rThigh,   -0.30f, 0.60f, 1);
                MAKE_JOINT(9, rThigh,  rShin,    -0.70f, 0.00f, 1);
#else // Debug player
                MAKE_JOINT(0, head,    chest,    -0.15f, 0.25f, 4.f/3.f);
                MAKE_JOINT(1, chest,   abdomin,  -0.05f, 0.20f, 1);
                MAKE_JOINT(2, chest,   lBicep,   -0.70f, 0.00f, -0.0f);
                MAKE_JOINT(3, lBicep,  lForearm, -0.00f, 0.80f, 1);
                MAKE_JOINT(4, chest,   rBicep,   -0.00f, 0.90f, -0.0f);
                MAKE_JOINT(5, rBicep,  rForearm, -0.00f, 0.80f, 1);
                MAKE_JOINT(6, abdomin, lThigh,   -0.30f, 0.00f, 1);
                MAKE_JOINT(7, lThigh,  lShin,    -0.70f, 0.00f, 1);
                MAKE_JOINT(8, abdomin, rThigh,   -0.00f, 0.60f, 1);
                MAKE_JOINT(9, rThigh,  rShin,    -0.70f, 0.00f, 1);
#endif
#undef MAKE_JOINT
            } else {
                if (ropeSwing) {
                    j->r1 = &state->playerBody.segments->lForearm;
                    j->r2 = &state->rects[
                        state->fixedRectCount +
                        state->stringSegmentRectCount - 1];
                    j->relPos1 = -V2(0, j->r1->h/2 - j->r1->w/2);
                    j->relPos2 = -V2(0, j->r2->h/2 - j->r2->w/2);
                    j->minTheta = -pi;
                    j->maxTheta = pi;
                } else j->enable = false;
            }
        }
        
        state->globalVel = {};
        state->globalPos = {};
        state->collision = true;
        
        state->isInitialised = true;
    }        
    
    // ---- INPUT ----
    GameControllerInput* controller = &input->controllers[0];
    
    f32 keyboardAccelScalar = 400.f;
    
    v2 keyboardAccel = {};
    if (controller->up.endedDown)
        keyboardAccel.y -= keyboardAccelScalar;
    if (controller->down.endedDown)
        keyboardAccel.y += keyboardAccelScalar;
    if (controller->left.endedDown)
        keyboardAccel.x += keyboardAccelScalar;
    if (controller->right.endedDown)
        keyboardAccel.x -= keyboardAccelScalar;

    if (keyboardAccel == V2(0,0)) {
        if (state->globalAccelTimer > 0)
            state->globalAccelTimer--;
    } else {
        if (state->globalAccelTimer < 100)
            state->globalAccelTimer++;
    }

    if (wasTapped(controller->aButton)) {
        /*PhysicsPlayer* pl = &state->players[0];
        for (int i = 0; i < state->particleCount; i++) {
            PhysicsParticle* p = &state->particles[i];
            v2 force = norm(p->p - pl->p) / mag(p->p - pl->p) * 100000000.f;
            p->v += dt/p->mass*force;
            }*/
        int i = 
            state->stringJointCount +
            state->playerJointCount;
        PhysicsJoint* j = &state->joints[i];
        j->enable = !j->enable;
        //state->rectCount--;
    }
    
    // ---- UPDATE AND RENDER ----

    state->globalForce = {};
    state->globalAccel = V2(0,-300.f);
    state->globalAccel += keyboardAccel;
    
    state->globalVel = state->globalAccel/5.f*(f32)state->globalAccelTimer/100.f;
    state->globalPos += dt*state->globalVel;

    //renderBackground(buffer);
    renderWeirdGradient(buffer, (int)state->globalPos.x, (int)state->globalPos.y);
    //renderRectangle(buffer, 2.f, (f32)buffer->height, V2(buffer->width/2.f,buffer->height/2.f), 0.f, 0x00FF00FF);

    int stepCount = awesomePhysics ? 50 : 1;
    dt /= (float)stepCount;
    for (int step = 0; step < stepCount; step++) {
        
        // --- SIMULATE ---
        for (int i = 0; i < state->rectCount; i++) {
            PhysicsRect* r = &state->rects[i];

#if 0
            // Compute vertices anticlockwise
            Polygon pRect;
            computeRectVertices(r, &pRect);
            v2* verts = pRect.verts;
            
            f32 forceStreamX = buffer->width/2.f;

            f32 maxX = -FLT_MAX;
            f32 minX = FLT_MAX;
            for (int i=0;i<4;i++) {
                maxX = max(maxX,verts[i].x);
                minX = min(minX,verts[i].x);
            }

            if (minX < forceStreamX && forceStreamX < maxX) {
                // Within stream
                v2 offset;
                offset.x = forceStreamX - r->p.x;
                offset.y = 0;
                v2 force = V2(0.f, 20000.f);
                f32 torque = dot(perp(offset), force);
                r->angularVel += dt*torque / r->momentOfInertia;
                r->v += dt*force / r->mass;
            }
#endif
            if (r->fixed) {
                r->v = V2(0,0);
                r->angularVel = 0;
                r->mass = FLT_MAX;
                r->momentOfInertia = FLT_MAX;
            } else {
                r->v += dt*state->globalForce/r->mass;
                r->v += dt*state->globalAccel;
                //r->p += dt*r->v;
                //r->angularVel *= 1.f - 1.f*dt;
                //r->v *= 1.f - 1.f*dt;
                //r->angle += dt*r->angularVel;
                if (awesomePhysics) {
                    if (r->p.x < 0 && r->v.x < 0) {
                        r->p.x = 0;
                        r->v.x *= -0.9f;
                    }
                    if (r->p.x > buffer->width && r->v.x > 0) {
                        r->p.x = (f32)buffer->width;
                        r->v.x *= -0.9f;
                    }
                    if (r->p.y < 0 && r->v.y < 0) {
                        r->p.y = 0;
                        r->v.y *= -0.9f;
                    }

                    if (r->p.y > buffer->height && r->v.y > 0) {
                        r->p.y = (f32)buffer->height;
                        r->v.y *= -0.9f;
                    }
                }
            }
            
            renderRectangle(buffer, r, r->colour);
        }
        
        for (int i = 0; i < state->playerCount; i++) {
            PhysicsPlayer* pl = &state->players[i];
            pl->v += dt*state->globalForce/pl->mass * (awesomePhysics ? -10000.f : 1.f);
            pl->v += dt*state->globalAccel * (awesomePhysics ? -10000.f : 1.f);
            pl->p += dt*pl->v;

            if (pl->p.x < 0 && pl->v.x < 0) {
                pl->p.x = 0;
                pl->v.x *= -0.9f;
            }
            if (pl->p.x > buffer->width && pl->v.x > 0) {
                pl->p.x = (f32)buffer->width;
                pl->v.x *= -0.9f;
            }
            if (pl->p.y < 0 && pl->v.y < 0) {
                pl->p.y = 0;
                pl->v.y *= -0.9f;
            }
            if (pl->p.y > buffer->height && pl->v.y > 0) {
                pl->p.y = (f32)buffer->height;
                pl->v.y *= -0.9f;
            }

            pl->v *= 1.f - (awesomePhysics ? 10.f : 0.01f) * dt;
            renderPlayer(buffer, (int)pl->p.x, (int)pl->p.y, (f32)(step+1)/stepCount);
        }

        for (int i = 0; i < state->particleCount; i++) {
            PhysicsParticle* p = &state->particles[i];
            p->v += dt*state->globalForce/p->mass;
            p->v += dt*state->globalAccel;
            p->p += dt*p->v;

            if (p->p.x < 0 && p->v.x < 0) {
                p->p.x = 0;
                p->v.x *= -0.9f;
            }
            if (p->p.x > buffer->width && p->v.x > 0) {
                p->p.x = (f32)buffer->width;
                p->v.x *= -0.9f;
            }
            if (p->p.y < 0 && p->v.y < 0) {
                p->p.y = 0;
                p->v.y *= -0.9f;
            }
            if (p->p.y > buffer->height && p->v.y > 0) {
                p->p.y = (f32)buffer->height;
                p->v.y *= -0.9f;
            }

            p->v *= 1.f - 0.2f*dt;
            
            if (step == stepCount-1) {
                renderSquare(buffer, (int)p->p.x, (int)p->p.y, 20);
            }
            
            // Collide with the players
            for (int j = 0; j < state->playerCount; j++) {
                PhysicsPlayer* pl = &state->players[j];

                v2 force = {};
                if (awesomePhysics) {
                    force = norm(p->p - pl->p) * 1000000000000.f
                        / sqr(sqr(mag(p->p - pl->p)));
                    p->v += dt/p->mass*force;
                }
                if (mag(pl->p - p->p) < 30 && awesomePhysics) {
                    // collide
                    //p->p.x = buffer->width/2.f;
                    //p->p.y = buffer->height/2.f;
                    p->v.x = (rand()-0.5f)*500.f;
                    p->v.y = (rand())*1000.f;
                    p->v += dt/p->mass*force;
                    pl->v *= 0.7f;
                }
            }
        }

        //TODO: Create collsion matrix for recently collided objects
        // --- COLLIDE ---
        if (state->collision) {
#if 0 // Official collision detection
           for (int i = 0; i < state->rectCount; i++) {
                PhysicsRect* r1 = &state->rects[i];
                Polygon p1; computeRectVertices(r1,&p1);
                for (int j = 0; j < i; j++) {
                    PhysicsRect* r2 = &state->rects[j];
                    Polygon p2; computeRectVertices(r2,&p2);
                    if (r1->fixed && r2->fixed) continue;
                    CollisionInfo collisionInfo;
                    if (doPolygonsIntersect(&p1,&p2,&collisionInfo)) {
                        resolveCollision(state,dt,collisionInfo,r1,r2);
#if 1
                        state->debugPause = true;
                        renderSquare(
                            buffer,
                            (int)collisionInfo.pos.x,
                            (int)collisionInfo.pos.y,
                            10);
                        for (int k = -5; k <= 5; k++) {
                            renderSquare(
                                buffer,
                                (int)(collisionInfo.pos.x+4*k*collisionInfo.normal.x),
                                (int)(collisionInfo.pos.y+4*k*collisionInfo.normal.y),
                                4);
                        }
#endif    
                    }
                }
            }
#else // Debug collision detection for constraints
           //TODO: Separate collisions into islands that don't collide: player1, player2, terrain
           for (int i = 0; i < state->rectCount; i++) {
               if (constraints && i == state->fixedRectCount)
                   i = state->fixedRectCount +
                       state->stringSegmentRectCount +
                       state->playerSegmentRectCount;
                PhysicsRect* r1 = &state->rects[i];
                Polygon p1; computeRectVertices(r1,&p1);
                for (int j = (constraints ? state->fixedRectCount : i+1); j < state->rectCount; j++) {
                    //NOTE: Temporary
                    if (j == 4 && i == 1 && constraints) continue;
                    PhysicsRect* r2 = &state->rects[j];
                    Polygon p2; computeRectVertices(r2,&p2);
                    if (r1->fixed && r2->fixed) continue;
                    CollisionInfo collisionInfo;
                    if (doPolygonsIntersect(&p1,&p2,&collisionInfo)) {
                        resolveCollision(state,dt,collisionInfo,r1,r2);    
                    }
                }
            }
#endif
        }
        
        int iterCount = 30;
        f32 h = dt/iterCount;
        for (int iter = 0; iter < iterCount; iter++) {
            for (int i = 0; i < state->rectCount; i++) {
                PhysicsRect* r = &state->rects[i];
                r->p += h * r->v;
                r->angle += h * r->angularVel;
            }
            // --- RESOLVE JOINT CONSTRAINTS ---
            for (int i = 0; i < state->jointCount; i++) {
                PhysicsJoint* j = &state->joints[i];
                if (!j->enable) continue;
                PhysicsRect* r1 = j->r1;
                PhysicsRect* r2 = j->r2;

                // --- ROTATION RESOLUTION ---
                f32 k_rot = -1.f / (1.f/r1->momentOfInertia + 1.f/r2->momentOfInertia);
                
                f32 theta = r1->angle - r2->angle;
                //TODO: this will slow down after a while, update the actual angles
                while (theta > pi) theta -= 2*pi;
                while (theta <= -pi) theta += 2*pi;
                
                f32 angVel = r1->angularVel - r2->angularVel;
                f32 rotBiasCoef = 0.0f;
                f32 rotBias = rotBiasCoef / dt * theta;
                f32 angImpulse = k_rot * (r1->angularVel - r2->angularVel + rotBias);
                
                if ((theta < j->minTheta && angVel < 0) ||
                    (theta > j->maxTheta && angVel > 0)) {
                    
                    if (!r1->fixed) r1->angularVel += angImpulse / r1->momentOfInertia;
                    if (!r2->fixed) r2->angularVel -= angImpulse / r2->momentOfInertia;
                }
                // --- POSITION RESOLUTION ---
                v2 rel1 = rotate(j->relPos1, r1->angle);
                v2 rel2 = rotate(j->relPos2, r2->angle);
                v2 pos1 = r1->p + rel1;
                v2 pos2 = r2->p + rel2;
                v2 pos = 0.5f*(pos1 + pos2);
                assert(!r1->fixed || !r2->fixed);
                bool fixed = (r1->fixed || r2->fixed);
                //TODO: can we do this using mass instead?
                //if (!r1->fixed) r1->p += 0.2f * (pos - pos1) * (fixed ? 2.f : 1.f);
                //if (!r2->fixed) r2->p += 0.2f * (pos - pos2) * (fixed ? 2.f : 1.f);
            
#if 0 //Impulse based resolution (too simplistic)
                v2 normal = norm(pos1-pos2);

                //TODO: Does this code apply, I copied it from the collision code and I cnba to recalculate
                f32 moment1 = (r1->fixed) ? 0 : sqr(cross(rel1, normal))/r1->momentOfInertia;
                f32 moment2 = (r2->fixed) ? 0 : sqr(cross(rel2, -normal))/r2->momentOfInertia;
                f32 momentSum = moment1 + moment2;
                f32 invMassSum = 0;
                if (!r1->fixed) invMassSum += 1.f/r1->mass;
                if (!r2->fixed) invMassSum += 1.f/r2->mass;
                f32 k = -1.f/(momentSum+invMassSum);
                f32 linVel1 = dot(normal, r1->v + dt * state->globalAccel);
                f32 linVel2 = dot(-normal, r2->v + dt * state->globalAccel);
                f32 angVel1 = cross(rel1, normal)*r1->angularVel;
                f32 angVel2 = cross(rel2, -normal)*r2->angularVel;
                f32 impulse = k * (linVel1 + linVel2 + angVel1 + angVel2);
                applyImpulsePair(state, dt, normal, impulse, pos, j->r1, j->r2);
#elif 0 //Force based resolution
                // impulse = k * (J' q' + J M^(-1) F_ext)
                // Where
                //   Factor k = -(J M^(-1) J^T)^(-1)
                //   Jacobi J = [deltaP | jacobiAngle1 | -deltaP | jacobiAngle2]
                //   Inertial matrix M = diag [m1 m1 I1 m2 m2 I2]
                //   State vector q = [p1 | angle1 | p2 | angle2]^T
                //   External force vector F_ext = [F1 | T1 | F2 | T2]^T
                v2 deltaP = pos1 - pos2;//norm(pos1 - pos2);
                v2 deltaV = //(
                    + r1->v + r1->angularVel * perp(rel1)
                    - r2->v - r2->angularVel * perp(rel2);//)
                // mag(pos1 - pos2);
                f32 jacobiAngle1 =  cross(rel1, deltaP);
                f32 jacobiAngle2 = -cross(rel2, deltaP);
                f32 linearThingy = sqrmag(deltaP) * (1.f / r1->mass + 1.f / r2->mass);
                f32 rotationalThingy =
                    + sqr(jacobiAngle1) / r1->momentOfInertia
                    + sqr(jacobiAngle2) / r2->momentOfInertia;
       
                f32 k = (linearThingy + rotationalThingy == 0 ? 0 : -1.f/(linearThingy + rotationalThingy));
                f32 jacobiAngVel1 =  cross(rel1, deltaV) - r1->angularVel * dot(rel1, deltaP);
                f32 jacobiAngVel2 = -cross(rel2, deltaV) + r2->angularVel * dot(rel2, deltaP);
                f32 _Jdot_qdot = dot(deltaV, r1->v - r2->v)
                    + jacobiAngVel1 * r1->angularVel
                    + jacobiAngVel2 * r2->angularVel;
                //TODO: Why only mass1?
                f32 _J_Minv_Fext = dot(deltaP, state->globalAccel) / r1->mass;
                f32 forceMultiplier = k * (_Jdot_qdot + _J_Minv_Fext);
                //forceMultiplier *= 0.8f;
                if (!r1->fixed) {
                    v2 accel1 = forceMultiplier * deltaP / r1->mass;
                    f32 angAccel1 = forceMultiplier * jacobiAngle1 / r1->momentOfInertia;
                    r1->v += dt*accel1;
                    r1->angle += dt*angAccel1;
                }
                if (!r2->fixed) {
                    v2 accel2 = forceMultiplier * deltaP / r2->mass;
                    f32 angAccel2 = forceMultiplier * jacobiAngle2 / r2->momentOfInertia;
                    r2->v += dt*accel2;
                    r2->angle += dt*angAccel2;
                }
#else // Other impulse resolver
                // impulse = k * (J q' + bias)
                // Where
                //   Bias b = beta / h * C
                //   Bias coef beta in [0,1] 
                //   Factor k = -(J M^(-1) J^T)^(-1)
                //   Jacobi J = [deltaP | jacobiAngle1 | -deltaP | jacobiAngle2]
                //   Inertial matrix M = diag [m1 m1 I1 m2 m2 I2]
                //   State vector q = [p1 | angle1 | p2 | angle2]^T
                //   External force vector F_ext = [F1 | T1 | F2 | T2]^T
                f32 m = 1;//mag(pos1-pos2);
                v2 deltaP = pos1 - pos2;//norm(pos1 - pos2);
                v2 deltaV = //(
                    + r1->v + r1->angularVel * perp(rel1)
                    - r2->v - r2->angularVel * perp(rel2);//)
                // mag(pos1 - pos2);
                f32 jacobiAngle1 =  cross(rel1, deltaP/m);
                f32 jacobiAngle2 = -cross(rel2, deltaP/m);
                f32 linearThingy = sqrmag(deltaP/m) * (1.f / r1->mass + 1.f / r2->mass);
                f32 rotationalThingy =
                    + sqr(jacobiAngle1) / r1->momentOfInertia
                    + sqr(jacobiAngle2) / r2->momentOfInertia;
       
                f32 k = (linearThingy + rotationalThingy == 0 ? 0 :
                         -1.f/(linearThingy + rotationalThingy));
                //f32 jacobiAngVel1 =  cross(rel1, deltaV) - r1->angularVel * dot(rel1, deltaP);
                //f32 jacobiAngVel2 = -cross(rel2, deltaV) + r2->angularVel * dot(rel2, deltaP);
                f32 newVel = dot(deltaP/m, r1->v - r2->v)
                    + jacobiAngle1 * r1->angularVel
                    + jacobiAngle2 * r2->angularVel;
                //f32 _J_Minv_Fext = dot(deltaP, state->globalAccel) / r1->mass;
                f32 biasCoef = 0.6f; //TODO: tune
                f32 bias = biasCoef / dt * 0.5f * sqrmag(deltaP)/m;
                f32 impulseCoef = k * (newVel + bias);
                //if (fabsf(impulseCoef) > 1000) impulseCoef /= impulseCoef / 1000.f;
                v2 linDeltaV1 =  impulseCoef * deltaP/m / r1->mass;
                v2 linDeltaV2 = -impulseCoef * deltaP/m / r2->mass;
                f32 rotDelta1 = impulseCoef * jacobiAngle1 / r1->momentOfInertia;
                f32 rotDelta2 =+impulseCoef * jacobiAngle2 / r2->momentOfInertia;
                if (!r1->fixed) {
                    r1->v += linDeltaV1;
                    r1->angularVel += rotDelta1;
                }
                if (!r2->fixed) {
                    r2->v += linDeltaV2;
                    r2->angularVel += rotDelta2;
                }
            
                //forceMultiplier *= 0.8f;
                //applyImpulsePair(state, dt, norm(deltaP), impulse*mag(deltaP),pos,r1,r2);

#endif
#if 1 // Debug draw constraint positions
                state->debugPause = true;
                renderSquare(
                    buffer,
                    (int)pos.x,
                    (int)pos.y,
                    5);
                for (int dist = 0; dist < 0; dist++) {
                    renderSquare(
                        buffer,
                        (int)(pos.x + dist*sin(r1->angle - j->minTheta)),
                        (int)(pos.y - dist*cos(r1->angle - j->minTheta)),
                        4);
                    renderSquare(
                        buffer,
                        (int)(pos.x + dist*sin(r1->angle - j->maxTheta)),
                        (int)(pos.y - dist*cos(r1->angle - j->maxTheta)),
                        4);
                }
#endif
            }
        }
    }    
    //renderPlayer(buffer, state->playerX, state->playerY);
}
