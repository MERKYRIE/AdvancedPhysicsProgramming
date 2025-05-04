//
//  Scene.cpp
//
#include "Scene.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Broadphase.h"

/*
========================================================================================================

Models

========================================================================================================
*/

static const float w = 50;
static const float h = 25;

Vec3 g_boxGround[] = {
	Vec3(-w,-h, 0),
	Vec3(w,-h, 0),
	Vec3(-w, h, 0),
	Vec3(w, h, 0),

	Vec3(-w,-h,-1),
	Vec3(w,-h,-1),
	Vec3(-w, h,-1),
	Vec3(w, h,-1),
};

Vec3 g_boxWall0[] = {
	Vec3(-1,-h, 0),
	Vec3(1,-h, 0),
	Vec3(-1, h, 0),
	Vec3(1, h, 0),

	Vec3(-1,-h, 5),
	Vec3(1,-h, 5),
	Vec3(-1, h, 5),
	Vec3(1, h, 5),
};

Vec3 g_boxWall1[] = {
	Vec3(-w,-1, 0),
	Vec3(w,-1, 0),
	Vec3(-w, 1, 0),
	Vec3(w, 1, 0),

	Vec3(-w,-1, 5),
	Vec3(w,-1, 5),
	Vec3(-w, 1, 5),
	Vec3(w, 1, 5),
};

Vec3 g_boxUnit[] = {
	Vec3(-1,-1,-1),
	Vec3(1,-1,-1),
	Vec3(-1, 1,-1),
	Vec3(1, 1,-1),

	Vec3(-1,-1, 1),
	Vec3(1,-1, 1),
	Vec3(-1, 1, 1),
	Vec3(1, 1, 1),
};

static const float t = 0.25f;
Vec3 g_boxSmall[] = {
	Vec3(-t,-t,-t),
	Vec3(t,-t,-t),
	Vec3(-t, t,-t),
	Vec3(t, t,-t),

	Vec3(-t,-t, t),
	Vec3(t,-t, t),
	Vec3(-t, t, t),
	Vec3(t, t, t),
};

static const float l = 3.0f;
Vec3 g_boxBeam[] = {
	Vec3(-l,-t,-t),
	Vec3(l,-t,-t),
	Vec3(-l, t,-t),
	Vec3(l, t,-t),

	Vec3(-l,-t, t),
	Vec3(l,-t, t),
	Vec3(-l, t, t),
	Vec3(l, t, t),
};

Vec3 g_boxPlatform[] = {
	Vec3(-l,-l,-t),
	Vec3(l,-l,-t),
	Vec3(-l, l,-t),
	Vec3(l, l,-t),

	Vec3(-l,-l, t),
	Vec3(l,-l, t),
	Vec3(-l, l, t),
	Vec3(l, l, t),
};

static const float t2 = 0.25f;
static const float w2 = t2 * 2.0f;
static const float h3 = t2 * 4.0f;
Vec3 g_boxBody[] = {
	Vec3(-t2,-w2,-h3),
	Vec3(t2,-w2,-h3),
	Vec3(-t2, w2,-h3),
	Vec3(t2, w2,-h3),

	Vec3(-t2,-w2, h3),
	Vec3(t2,-w2, h3),
	Vec3(-t2, w2, h3),
	Vec3(t2, w2, h3),
};

static const float h2 = 0.25f;
Vec3 g_boxLimb[] = {
	Vec3(-h3,-h2,-h2),
	Vec3(h3,-h2,-h2),
	Vec3(-h3, h2,-h2),
	Vec3(h3, h2,-h2),

	Vec3(-h3,-h2, h2),
	Vec3(h3,-h2, h2),
	Vec3(-h3, h2, h2),
	Vec3(h3, h2, h2),
};

Vec3 g_boxHead[] = {
	Vec3(-h2,-h2,-h2),
	Vec3(h2,-h2,-h2),
	Vec3(-h2, h2,-h2),
	Vec3(h2, h2,-h2),

	Vec3(-h2,-h2, h2),
	Vec3(h2,-h2, h2),
	Vec3(-h2, h2, h2),
	Vec3(h2, h2, h2),
};

Vec3 g_diamond[7 * 8];
void FillDiamond()
{
	Vec3 pts[4 + 4];
	pts[0] = Vec3(0.1f, 0, -1);
	pts[1] = Vec3(1, 0, 0);
	pts[2] = Vec3(1, 0, 0.1f);
	pts[3] = Vec3(0.4f, 0, 0.4f);

	const float pi = acosf(-1.0f);
	const Quat quatHalf(Vec3(0, 0, 1), 2.0f * pi * 0.125f * 0.5f);
	pts[4] = Vec3(0.8f, 0, 0.3f);
	pts[4] = quatHalf.RotatePoint(pts[4]);
	pts[5] = quatHalf.RotatePoint(pts[1]);
	pts[6] = quatHalf.RotatePoint(pts[2]);

	const Quat quat(Vec3(0, 0, 1), 2.0f * pi * 0.125f);
	int idx = 0;
	for (int i = 0; i < 7; i++) {
		g_diamond[idx] = pts[i];
		idx++;
	}

	Quat quatAccumulator;
	for (int i = 1; i < 8; i++) {
		quatAccumulator = quatAccumulator * quat;
		for (int pt = 0; pt < 7; pt++) {
			g_diamond[idx] = quatAccumulator.RotatePoint(pts[pt]);
			idx++;
		}
	}
}

void AddStandardSandBox(std::vector<Body>& bodies) {
	Body body;

	body.position = Vec3(0, 0, 0);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity.Zero();
	body.angularVelocity.Zero();
	body.inverseMass = 0.0f;
	body.elasticity = 0.5f;
	body.friction = 0.5f;
	body.shape = new ShapeBox(g_boxGround, sizeof(g_boxGround) / sizeof(Vec3));
	bodies.push_back(body);

	body.position = Vec3(50, 0, 0);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity.Zero();
	body.angularVelocity.Zero();
	body.inverseMass = 0.0f;
	body.elasticity = 0.5f;
	body.friction = 0.0f;
	body.shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	bodies.push_back(body);

	body.position = Vec3(-50, 0, 0);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity.Zero();
	body.angularVelocity.Zero();
	body.inverseMass = 0.0f;
	body.elasticity = 0.5f;
	body.friction = 0.0f;
	body.shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	bodies.push_back(body);

	body.position = Vec3(0, 25, 0);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity.Zero();
	body.angularVelocity.Zero();
	body.inverseMass = 0.0f;
	body.elasticity = 0.5f;
	body.friction = 0.0f;
	body.shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	bodies.push_back(body);

	body.position = Vec3(0, -25, 0);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity.Zero();
	body.angularVelocity.Zero();
	body.inverseMass = 0.0f;
	body.elasticity = 0.5f;
	body.friction = 0.0f;
	body.shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	bodies.push_back(body);
}

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for (int i = 0; i < bodies.size(); i++) {
		delete bodies[i].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for (int i = 0; i < bodies.size(); i++) {
		delete bodies[i].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;

	body.position = Vec3(10, 0, 3);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity = Vec3(-100, 0, 0);
	body.angularVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.inverseMass = 1.0f;
	body.elasticity = 0.5f;
	body.friction = 0.5f;
	body.shape = new ShapeSphere(0.5f);
	bodies.push_back(body);

	body.position = Vec3(-10, 0, 3);
	body.orientation = Quat(0, 0, 0, 1);
	body.linearVelocity = Vec3(100, 0, 0);
	body.angularVelocity = Vec3(0, 10, 0);
	body.inverseMass = 1.0f;
	body.elasticity = 0.5f;
	body.friction = 0.5f;
	body.shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
	bodies.push_back(body);

	AddStandardSandBox(bodies);
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec) 
{
	// Gravity
	for (int i = 0; i < bodies.size(); ++i) 
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);
	}

	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);

	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)alloca(sizeof(Contact) * maxContacts);
	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];

		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f) continue;

		Contact contact;
		if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}

	// Sort times of impact
	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(Contact), Contact::CompareContact);
	}

	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i) 
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;

		// Position update
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].Update(dt);
		}

		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}

	// Other physics behavirous, outside collisions. 
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].Update(timeRemaining);
		}
	}
}