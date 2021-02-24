#include "kfighter.h"
#include "kfighter_util.h"

internal void makePlayer(
	modified GameState* state,
	out Player* pl,
	in GameOffscreenBuffer* buffer) {

	CollisionIsland* ci = &state->collisionIslandArr[
		state->collisionIslandCount++];
	ci->enable = true;
	ci->objCount = playerSegmentCount;
	ci->objs = &state->objArr[state->objCount];

	pl->segments = (PlayerSegments*)ci->objs;
	PhysicsObj* r;
	f32 margin = 150;
	f32 pX = rand(state)*(buffer->width-2*margin) + margin;
	f32 pY = rand(state)*(buffer->height-2*margin) + margin;
	for (int i = 0; i < ci->objCount; i++) {
		assert(state->objCount < ARRAY_COUNT(state->objArr));
		r = &state->objArr[state->objCount++];
		r->fixed = false;
		r->enableFriction = true;
		r->p.x = pX + (rand(state)-.5f)*50.f;
		r->p.y = pY + (rand(state)-.5f)*50.f;
		r->v.x = (rand(state)-0.5f)*20.f;
		r->v.y = (rand(state)-0.5f)*20.f;
		r->lastV = r->v;
		r->angle = (rand(state)*2*pi);
		r->angularVel = (rand(state)*pi/4.f);
		if (pl == &state->playerArr[0]) {
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
				if (pl == &state->playerArr[0]) {
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
	assert(state->jointCount + playerJointCount
		<= ARRAY_COUNT(state->jointArr));
	pl->joints = (PlayerJoints*)&state->jointArr[state->jointCount];
	state->jointCount += playerJointCount;

#define MAKE_JOINT(joint, part1, part2, min, max, vOffsetFactor) \
	{															\
		PhysicsJoint* j = &pl->joints->joint;					\
		j->r1 = &seg->part1;									 \
		j->r2 = &seg->part2;									 \
		j->relPos1 = -V2(										\
			0,												   \
			j->r1->h/2*(vOffsetFactor) - j->r1->w/2);			\
		j->relPos2 = +V2(0, j->r2->h/2 - j->r2->w/2);			\
		j->minTheta = (min)*pi;								  \
		j->maxTheta = (max)*pi;								  \
		j->targetAngVel = 0;									 \
		j->targetAngle = 0;									  \
		j->pidIntegralTerm = 0;								  \
		j->enableMotor = false;								  \
		j->enablePID = false;									\
		j->enable = true;										\
	}

	MAKE_JOINT(neck,	  head,	chest,	-0.30f, 0.40f, 5.f/3.f);
	MAKE_JOINT(back,	  chest,   abdomen,  -0.05f, 0.20f, 1.f);
	MAKE_JOINT(lShoulder, chest,   lBicep,   -0.70f, 0.90f, -0.0f);
	MAKE_JOINT(lElbow,	lBicep,  lForearm, -0.00f, 0.80f, 1.f);
	MAKE_JOINT(rShoulder, chest,   rBicep,   -0.70f, 0.90f, -0.0f);
	MAKE_JOINT(rElbow,	rBicep,  rForearm, -0.00f, 0.80f, 1.f);
	MAKE_JOINT(lHip,	  abdomen, lThigh,   -0.30f, 0.60f, 1.f);
	MAKE_JOINT(lKnee,	 lThigh,  lShin,	-0.70f, 0.00f, 1.f);
	MAKE_JOINT(rHip,	  abdomen, rThigh,   -0.30f, 0.60f, 1.f);
	MAKE_JOINT(rKnee,	 rThigh,  rShin,	-0.70f, 0.00f, 1.f);

#undef MAKE_JOINT

	// --- ADDITIONAL VARIABLES ---
	pl->currentPose = randIntBetween(state, -1, state->poseCount);
	pl->torque = 0;
	pl->rPunching = false;
	pl->lPunching = false;
	pl->strideWheelRadius = 75.f;
	pl->strideWheelAngle = 0.f;
	//pl->prevPose = state->runningLPassPose;
	//pl->nextPose = state->runningLReachPose;
	pl->direction = DIRECTION_RIGHT;
}

internal void
lrFlipPlayerPose(
	in_array PlayerPose* poses,
	in_index(poses) int inIndex,
	out_index(poses) int outIndex) {

	PlayerPose* outPose = &poses[outIndex];
	PlayerPose* inPose = &poses[inIndex];

	outPose->lShoulder = inPose->rShoulder;
	outPose->lElbow = inPose->rElbow;
	outPose->rShoulder = inPose->lShoulder;
	outPose->rElbow = inPose->lElbow;

	outPose->lHip = inPose->rHip;
	outPose->lKnee = inPose->rKnee;
	outPose->rHip = inPose->lHip;
	outPose->rKnee = inPose->lKnee;
}

//NOTE: This requires the first player to have already been made since
//it uses joint limits to specify poses like the ball.
internal void makePlayerPoses(modified GameState* state) {
	PlayerPose* pose;


	// --- IDLE POSES ---

	pose = &state->poseArr[PLAYER_POSE_DEFAULT];
	for (int i = 0; i < playerJointCount; i++) {
		pose->joints[i].angle = 0.f;
		pose->joints[i].applicationFactor = 1.f;
	}

	pose = &state->poseArr[PLAYER_POSE_BALL];
	Player* pl = &state->playerArr[0];
	for (int i = 0; i < playerJointCount; i++) {
		pose->joints[i].angle = pl->joints->joints[i].maxTheta;
		pose->joints[i].applicationFactor = 1.f;
	}
	pose->lShoulder.angle = 0.1f*pi;
	pose->rShoulder.angle = 0.1f*pi;
	pose->lKnee.angle = pl->joints->lKnee.minTheta;
	pose->rKnee.angle = pl->joints->rKnee.minTheta;

	pose = &state->poseArr[PLAYER_POSE_READY];
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


	// --- PUNCH POSES ---

	pose = &state->poseArr[PLAYER_POSE_PUNCH_PREP];
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
	pose->lHip.angle = 4*pi/12.f + pi/5.f;
	pose->lHip.applicationFactor = 0.1f;
	pose->lKnee.angle = -3*pi/8.f;
	pose->lKnee.applicationFactor = 0.1f;
	pose->rHip.angle = 0.f;
	pose->rHip.applicationFactor = 0.1f;
	pose->rKnee.angle = 0.f;
	pose->rKnee.applicationFactor = 0.1f;

	pose = &state->poseArr[PLAYER_POSE_PUNCH_EXTEND];
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
	pose->lHip.angle = 4*pi/12.f + pi/5.f;;
	pose->lHip.applicationFactor = 0.1f;
	pose->lKnee.angle = -3*pi/8.f;
	pose->lKnee.applicationFactor = 0.1f;
	pose->rHip.angle = 0.f;
	pose->rHip.applicationFactor = 0.1f;
	pose->rKnee.angle = 0.f;
	pose->rKnee.applicationFactor = 0.1f;


	// --- RUNNING POSES ---

	pose = &state->poseArr[PLAYER_POSE_RUN_L_PASS];
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

	pose = &state->poseArr[PLAYER_POSE_RUN_L_REACH];
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

	lrFlipPlayerPose(state->poseArr,
					 PLAYER_POSE_RUN_L_PASS,
					 PLAYER_POSE_RUN_R_PASS);
	lrFlipPlayerPose(state->poseArr,
					 PLAYER_POSE_RUN_L_REACH,
					 PLAYER_POSE_RUN_R_REACH);


	// --- WALKING POSES ---

	pose = &state->poseArr[PLAYER_POSE_WALK_L_PASS];
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

	pose = &state->poseArr[PLAYER_POSE_WALK_L_REACH];
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

	lrFlipPlayerPose(state->poseArr,
					 PLAYER_POSE_WALK_L_PASS,
					 PLAYER_POSE_WALK_R_PASS);
	lrFlipPlayerPose(state->poseArr,
					 PLAYER_POSE_WALK_L_REACH,
					 PLAYER_POSE_WALK_R_REACH);
}

internal PoseJoint interpolateJoint(f32 t, in PoseJoint* j1, in PoseJoint* j2) {
	assert(j1->applicationFactor && j2->applicationFactor);

	PoseJoint result;
	f32 t1 = (1-t) * j1->applicationFactor;
	f32 t2 = t * j2->applicationFactor;
	result.applicationFactor = t1 + t2;
	result.angle = (t1 * j1->angle + t2 * j2->angle) / (t1+t2);
	return result;
}

internal PoseJoint overlayJoint(f32 t, in PoseJoint* j1, in PoseJoint* j2) {
	//assert(j1->applicationFactor && j2->applicationFactor);

	PoseJoint result;
	f32 t1 = (1-t) * j1->applicationFactor;
	f32 t2 = t * j2->applicationFactor;
	result.applicationFactor = t1 + t2;
	result.angle = (t1 * j1->angle + t2 * j2->angle) / (t1+t2);
	return result;
}

internal void
updatePlayer(
	modified Player* pl,
	in_array PlayerPose* poseArr,
	f32 dt) {

	// Match pose
	if (~pl->currentPose) { // If matching a specific pose
		PlayerPose* pose = &poseArr[pl->currentPose];
		for (int i = 0; i < playerJointCount; i++) {
			f32 targetAngle = pose->joints[i].angle;
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
