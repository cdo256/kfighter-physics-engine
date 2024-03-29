#if !defined(KFIGHTER_PHYSICS_H)

global const int maxVerticesInPolygon = 16;

struct Polygon {
	int count;
	v2 center;
	v2 verts[maxVerticesInPolygon];
};

struct PhysicsObj {
	f32 w, h;
	v2 p;
	v2 v;
	v2 lastV;
	v2 accel;
	f32 angle;
	f32 angularVel;
	//TODO: Should I store inverse [mass and moment of inertia] instead?
	f32 mass;
	f32 momentOfInertia;
	bool fixed;
	bool enableFriction;

	//TODO: Does colour belong here?
	u32 colour;
};

struct CollisionManifold {
	int count;
	v2 pos[2];
	v2 normal;
	f32 depth;
	PhysicsObj *r1, *r2;
};

struct PhysicsJoint {
	PhysicsObj* r1;
	PhysicsObj* r2;
	v2 relPos1, relPos2;
	f32 minTheta, maxTheta;
	f32 maxTorque;
	f32 targetAngVel;
	f32 targetAngle;

	f32 pidLastError;
	f32 pidIntegralTerm;

	bool enableMotor;
	bool enablePID;
	bool enable;
};

//NOTE: Collision islands are collections of objects typically
//connected by joints or other constraints that shouldn't collide with
//each other.
struct CollisionIsland {
	bool enable;

	int objCount;
	PhysicsObj* objs;
};

#define KFIGHTER_PHYSICS_H
#endif
