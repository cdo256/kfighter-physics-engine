#if !defined(KFIGHTER_PLAYER_H)

global const int playerSegmentCount = 11;
global const int playerJointCount = 10;

union PlayerSegments {
	PhysicsRect segments[playerSegmentCount];
	struct {
		PhysicsRect head;
		PhysicsRect chest;
		PhysicsRect abdomen;
		PhysicsRect lBicep;
		PhysicsRect lForearm;
		PhysicsRect rBicep;
		PhysicsRect rForearm;
		PhysicsRect lThigh;
		PhysicsRect lShin;
		PhysicsRect rThigh;
		PhysicsRect rShin;
	};
};

union PlayerJoints {
	PhysicsJoint joints[playerJointCount];
	struct {
		PhysicsJoint neck;
		PhysicsJoint back;
		PhysicsJoint lShoulder;
		PhysicsJoint lElbow;
		PhysicsJoint rShoulder;
		PhysicsJoint rElbow;
		PhysicsJoint lHip;
		PhysicsJoint lKnee;
		PhysicsJoint rHip;
		PhysicsJoint rKnee;
	};
};

struct PoseJoint {
	f32 angle;
	f32 applicationFactor; // 1 is apply fully, 0 is don't apply (used in interpolation)
};

enum PlayerPoses {
	PLAYER_POSE_NONE=-1,
	PLAYER_POSE_DEFAULT,
	PLAYER_POSE_READY,
	PLAYER_POSE_BALL,
	PLAYER_POSE_PUNCH_PREP,
	PLAYER_POSE_PUNCH_EXTEND,
	PLAYER_POSE_RUN_L_PASS,
	PLAYER_POSE_RUN_L_REACH,
	PLAYER_POSE_RUN_R_PASS,
	PLAYER_POSE_RUN_R_REACH,
	PLAYER_POSE_WALK_L_PASS,
	PLAYER_POSE_WALK_L_REACH,
	PLAYER_POSE_WALK_R_PASS,
	PLAYER_POSE_WALK_R_REACH,
	PLAYER_POSE_ENUM_COUNT
};

struct PlayerPose {
	union {
		PoseJoint joints[playerJointCount];
		struct {
			PoseJoint neck;
			PoseJoint back;
			PoseJoint lShoulder;
			PoseJoint lElbow;
			PoseJoint rShoulder;
			PoseJoint rElbow;
			PoseJoint lHip;
			PoseJoint lKnee;
			PoseJoint rHip;
			PoseJoint rKnee;
		};
	};
};

enum Direction {DIRECTION_LEFT, DIRECTION_RIGHT};

struct Player {
	PlayerSegments* segments;
	PlayerJoints* joints;
	Direction direction;

	//TODO: The following is all legacy stuff and needs to go (or be
	//completely redone) at some point
	s32 currentPose;
	//PlayerPose *prevPose, *nextPose;
	f32 strideWheelRadius;
	f32 strideWheelAngle;

	bool lPunching, rPunching;
	f32 rPunchTimer, lPunchTimer;
	v2 lPunchTarget, rPunchTarget;
	f32 torque;
};

#define KFIGHTER_PLAYER_H
#endif
