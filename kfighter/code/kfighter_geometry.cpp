/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

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

struct SupportVectors {
    int count;
    int indices[2];
};

#if 0
internal SupportVectors supportVectors(Polygon* p, v2 vec) {
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
}
#endif

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

internal bool doPolygonsIntersect(Polygon* a, Polygon* b, CollisionManifold* manifold) {
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
            
            if (maxA < minB || maxB < minA) {
                manifold->count = 0;
                return false;
            }

            f32 overlap = min(maxA-minB,maxB-minA) / mag(normal);
            if (overlap < minOverlap) {
                manifold->normal = normal;
                minOverlap = overlap;
                vertexShape = (p == a ? b : a);
                minShape = (maxA-minB > maxB-minA ? a : b);
            }
        }
    }

    f32 sign = (vertexShape == minShape ? -1.f : 1.f);
    manifold->count = 1;
    manifold->normal *= sign;
    int supportIndex;
    if (vertexShape == a || vertexShape == b)
        supportIndex = supportVector(vertexShape, manifold->normal);
    else int x=5;
    v2 supportVertex = vertexShape->verts[supportIndex];
    manifold->depth = minOverlap;
    manifold->pos[0] = supportVertex;// + sign/2.f * minOverlap * manifold->normal / sqrmag(manifold->normal);
    //NOTE: Normal is always pointing from a to b
    manifold->normal = (vertexShape == a ? -1.f : 1.f) * norm(manifold->normal);
    return true;
}

internal v2 getJointPosition(PhysicsJoint* j) {
    v2 rel1 = rotate(j->relPos1, j->r1->angle);
    v2 rel2 = rotate(j->relPos2, j->r2->angle);
    v2 pos1 = j->r1->p + rel1;
    v2 pos2 = j->r2->p + rel2;
    v2 pos = 0.5f*(pos1 + pos2);
    return pos;

}

inline f32 getJointAngle(PhysicsJoint* j) {
    return j->r1->angle - j->r2->angle;
}
