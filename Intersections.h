#pragma once
#include "Body.h"
#include "Shape.h"
#include "Contact.h"

class Intersections
{
public:
	static bool Intersect(Body& a, Body& b, const float dt, Contact& contact);

	static bool RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float sphereRadius, float& t1, float& t2);

	static bool SphereSphereDynamic(const ShapeSphere& shapeA, const ShapeSphere& shapeB,
		const Vec3& posA, const Vec3& posB, const Vec3& velA, const Vec3& velB,
		const float dt, Vec3& ptOnA, Vec3& ptOnB, float& timeOfImpact);

 static bool SphereSphereStatic(const ShapeSphere* sphereA, const ShapeSphere* sphereB,
		const Vec3& posA, const Vec3& posB, Vec3& ptOnA, Vec3& ptOnB);

	static bool Intersect(Body* a, Body* b, Contact& contact);

 static bool ConservativeAdvance(Body& bodyA, Body& bodyB, float dt, Contact& contact);

};

