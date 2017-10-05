/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

inline void computeMassAndMomentOfInertia(PhysicsRect* r) {
    if (r->fixed) {
        r->mass = FLT_MAX;
        r->momentOfInertia = FLT_MAX;
    } else {
        r->mass = r->w*r->h*0.01f;
        r->momentOfInertia = r->mass *
            (sqr(r->w)+sqr(r->h))/12.f;
    }
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
    CollisionManifold* manifold) {

    for (int i = 0; i < manifold->count; i++) {
        v2 pos = manifold->pos[i];
        v2 normal = manifold->normal;
        f32 depth = manifold->depth;
        PhysicsRect* r1 = manifold->r1;
        PhysicsRect* r2 = manifold->r2;
        
        normal = norm(normal);
        v2 rel1 = pos - r1->p;
        v2 rel2 = pos - r2->p;
        v2 vel1 = r1->v + r1->angularVel * perp(rel1);
        v2 vel2 = r2->v + r2->angularVel * perp(rel2);

        bool fixed = (r1->fixed || r2->fixed);
        if (!r1->fixed) r1->p += depth*normal / (fixed ? 1.f : 2.f);
        if (!r2->fixed) r2->p -= depth*normal / (fixed ? 1.f : 2.f); 

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
        f32 effectiveMass = -1.f/(momentSum+invMassSum);
        f32 linVel1 = dot(normal, r1->v + dt * state->globalAccel);
        f32 linVel2 = dot(-normal, r2->v + dt * state->globalAccel);
        f32 angVel1 = cross(rel1, normal)*r1->angularVel;
        f32 angVel2 = cross(rel2, -normal)*r2->angularVel;
        f32 impulse = effectiveMass * (linVel1 + linVel2 + angVel1 + angVel2);

        if (impulse >= 0) applyImpulsePair(state, dt, normal, impulse, pos, r1, r2);
        if (state->enableFriction && r1->enableFriction && r2->enableFriction) {
            v2 tangent = perp(normal);
            f32 moment1 = (r1->fixed) ? 0 : sqr(cross(rel1, tangent))/r1->momentOfInertia;
            f32 moment2 = (r2->fixed) ? 0 : sqr(cross(rel2, -tangent))/r2->momentOfInertia;
            f32 momentSum = moment1 + moment2;
            f32 invMassSum = 0;
            if (!r1->fixed) invMassSum += 1.f/r1->mass;
            if (!r2->fixed) invMassSum += 1.f/r2->mass;
            f32 effectiveMass = -1.f/(momentSum+invMassSum);
            f32 linVel1 = dot(tangent, r1->v + dt * state->globalAccel);
            f32 linVel2 = dot(-tangent, r2->v + dt * state->globalAccel);
            f32 angVel1 = cross(rel1, tangent)*r1->angularVel;
            f32 angVel2 = cross(rel2, -tangent)*r2->angularVel;
            f32 impulse = effectiveMass * (linVel1 + linVel2 + angVel1 + angVel2);
            applyImpulsePair(state, dt, tangent, state->frictionCoef*impulse, pos, r1, r2);
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
    }
}

internal void resolveJointConstraint(GameState* state, PhysicsJoint* j, f32 dt) {
    if (!j->enable) return;
    PhysicsRect* r1 = j->r1;
    PhysicsRect* r2 = j->r2;

    f32 theta = r1->angle - r2->angle;
    theta = normAngle(theta);
    f32 angVel = r1->angularVel - r2->angularVel;
    f32 effectiveMass = -1.f / (1.f/r1->momentOfInertia + 1.f/r2->momentOfInertia);
    
    // --- MOTOR ---
    if (state->enableMotor && j->enableMotor) {
        f32 targetAngVel = j->targetAngVel;
        f32 angImpulse = effectiveMass
            * (angVel - targetAngVel);
        
        if (state->enablePIDJoints && j->enablePID) {
            //TODO: Tune these
            //TODO: Prevent integral windup:
            // https://en.wikipedia.org/wiki/Integral_windup
            //NOTE: Cranking up ki leads to some kung-fu-like results
            //f32 kp=300.f,ki=10000.f,kd=0;
            f32 kp=300000.f,ki=5000.f,kd=1000.f;
            //f32 kp=70000.f,ki=2000.f,kd=10.f;
            f32 error = j->targetAngle - theta;
            f32 integral = j->pidIntegralTerm += dt*error;
            f32 derivative = (error - j->pidLastError)/dt;
            angImpulse = kp*error + ki*integral + kd*derivative;
            j->pidLastError = error;
            
            /*
            j->targetAngVel = 0.01f/dt*error;
            j->targetAngVel -= 0.9f*angVel;
            j->targetAngVel += 0.f*j->pidIntegralTerm;
            */
        }
        
        f32 maxImpulse = 10000000.f;//state->maxMotorTorque*dt;
        angImpulse = bound(angImpulse, -maxImpulse, maxImpulse);
        if (!r1->fixed) r1->angularVel += angImpulse / r1->momentOfInertia;
        if (!r2->fixed) r2->angularVel -= angImpulse / r2->momentOfInertia;
    }

    // --- JOINT FRICTION ---
    f32 effectiveMomentOfInertia = -1.f /
        (1.f/r1->momentOfInertia + 1.f/r2->momentOfInertia);
    f32 angImpulse = effectiveMomentOfInertia * angVel;

    if (!r1->fixed)
        r1->angularVel +=
            state->jointFrictionCoef * angImpulse / r1->momentOfInertia;
    if (!r2->fixed)
        r2->angularVel -=
            state->jointFrictionCoef * angImpulse / r2->momentOfInertia;
    
    // --- VELOCITY RESOLUTION ---
    v2 rel1 = rotate(j->relPos1, r1->angle);
    v2 rel2 = rotate(j->relPos2, r2->angle);
    v2 pos1 = r1->p + rel1;
    v2 pos2 = r2->p + rel2;
    v2 pos = 0.5f*(pos1 + pos2);
    assert(!r1->fixed || !r2->fixed);
    bool fixed = (r1->fixed || r2->fixed);
                
    // impulse = k * (J q' + bias)
    // Where
    //   Bias b = beta / h * C
    //   Bias coef beta in [0,1] 
    //   Factor k = -(J M^(-1) J^T)^(-1)
    //   Jacobi J = [deltaP | jacobiAngle1 | -deltaP | jacobiAngle2]
    //   Inertial matrix M = diag [m1 m1 I1 m2 m2 I2]
    //   State vector q = [p1 | angle1 | p2 | angle2]^T
    //   External force vector F_ext = [F1 | T1 | F2 | T2]^T

    v2 deltaP = pos1 - pos2;
    v2 deltaV = 
        + r1->v + r1->angularVel * perp(rel1)
        - r2->v - r2->angularVel * perp(rel2);
                
    f32 jacobiAngle1 =  cross(rel1, deltaP);
    f32 jacobiAngle2 = -cross(rel2, deltaP);
    f32 linearThingy = sqrmag(deltaP) * (1.f / r1->mass + 1.f / r2->mass);
    f32 rotationalThingy =
        + sqr(jacobiAngle1) / r1->momentOfInertia
        + sqr(jacobiAngle2) / r2->momentOfInertia;
       
    f32 effectiveMass2 = (linearThingy + rotationalThingy == 0 ? 0 :
             -1.f/(linearThingy + rotationalThingy));
                
    f32 newVel = dot(deltaP, r1->v - r2->v)
        + jacobiAngle1 * r1->angularVel
        + jacobiAngle2 * r2->angularVel;
    f32 bias = state->jointPositionalBiasCoef / dt * 0.5f * sqrmag(deltaP);
    f32 impulseCoef = effectiveMass2 * (newVel + bias);
                
    v2 linDeltaV1 =  impulseCoef * deltaP / r1->mass;
    v2 linDeltaV2 = -impulseCoef * deltaP / r2->mass;
    f32 rotDelta1 =  impulseCoef * jacobiAngle1 / r1->momentOfInertia;
    f32 rotDelta2 = +impulseCoef * jacobiAngle2 / r2->momentOfInertia;
    if (!r1->fixed) {
        r1->v += linDeltaV1;
        r1->angularVel += rotDelta1;
    }
    if (!r2->fixed) {
        r2->v += linDeltaV2;
        r2->angularVel += rotDelta2;
    }

    // --- ROTATION RESOLUTION ---
    angVel = r1->angularVel - r2->angularVel;
    f32 rotBiasCoef = 0.f;
    f32 rotBias = rotBiasCoef / dt * theta;
    angImpulse = effectiveMass * (angVel + rotBias);
    if (state->enableRotationalConstraints) {
        if ((theta < j->minTheta && angVel < 0) ||
            (theta > j->maxTheta && angVel > 0)) {

            if (!r1->fixed) {
                r1->angularVel += angImpulse / r1->momentOfInertia;
            }
            if (!r2->fixed) {
                r2->angularVel -= angImpulse / r2->momentOfInertia;
            }
        }
        
        f32 thetaDiff = bound(theta,j->minTheta,j->maxTheta) - theta;
		if (thetaDiff > 1.f)
			normAngle(theta);

        if (!r1->fixed) r1->angle += thetaDiff * (r2->fixed ? 1.f : 0.5f);
        if (!r2->fixed) r2->angle -= thetaDiff * (r1->fixed ? 1.f : 0.5f);
    }
}
