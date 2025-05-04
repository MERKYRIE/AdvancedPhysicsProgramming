#include "Intersections.h"
#include "GJK.h"

bool Intersections::Intersect(Body& a, Body& b, const float dt, Contact& contact)
{
	contact.a = &a;
	contact.b = &b;
	const Vec3 ab = b.position - a.position;
	contact.normal = ab;
	contact.normal.Normalize();

	if (a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE && b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE)
 {
		ShapeSphere* sphereA = static_cast<ShapeSphere*>(a.shape);
		ShapeSphere* sphereB = static_cast<ShapeSphere*>(b.shape);

		Vec3 posA = a.position;
		Vec3 posB = b.position;
		Vec3 valA = a.linearVelocity;
		Vec3 velB = b.linearVelocity;

		if (Intersections::SphereSphereDynamic(*sphereA, *sphereB, posA, posB, valA, velB, dt,
			contact.ptOnAWorldSpace, contact.ptOnBWorldSpace, contact.timeOfImpact))
		{
			// Step bodies forward to get local space collision points
			a.Update(contact.timeOfImpact);
			b.Update(contact.timeOfImpact);

			// Convert world space contacts to local space
			contact.ptOnALocalSpace = a.WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
			contact.ptOnBLocalSpace = b.WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

			Vec3 ab = a.position - b.position;
			contact.normal = ab;
			contact.normal.Normalize();

			// Unwind time step
			a.Update(-contact.timeOfImpact);
			b.Update(-contact.timeOfImpact);

			// Calculate separation distance
			float r = ab.GetMagnitude() - (sphereA->radius + sphereB->radius);
			contact.separationDistance = r;
			return true;
		}
	}
 else
	{
		// Use GJK to perform conservative advancement
		bool result = ConservativeAdvance(a, b, dt, contact);
		return result;
	}

	return false;
}

bool Intersections::RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float sphereRadius, float& t0, float& t1)
{
	const Vec3& s = sphereCenter - rayStart;
	const float a = rayDir.Dot(rayDir);
	const float b = s.Dot(rayDir);
	const float c = s.Dot(s) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	const float inverseA = 1.0f / a;

	if (delta < 0) {
		// No solution
		return false;
	}

	const float deltaRoot = sqrtf(delta);
	t0 = (b - deltaRoot) * inverseA;
	t1 = (b + deltaRoot) * inverseA;

	return true;
}

bool Intersections::SphereSphereDynamic(const ShapeSphere& shapeA, const ShapeSphere& shapeB, 
	const Vec3& posA, const Vec3& posB, const Vec3& velA, const Vec3& velB, 
	const float dt, Vec3& ptOnA, Vec3& ptOnB, float& timeOfImpact)
{
	const Vec3 relativeVelocity = velA - velB;

	const Vec3 startPtA = posA;
	const Vec3 endPtA = startPtA + relativeVelocity * dt;
	const Vec3 rayDir = endPtA - startPtA;

	float t0 = 0;
	float t1 = 0;
	if (rayDir.GetLengthSqr() < 0.001f * 0.001f)
	{
		// Ray is too short, just check if already intersecting
		Vec3 ab = posB - posA;
		float radius = shapeA.radius + shapeB.radius + 0.001f;
		if (ab.GetLengthSqr() > radius * radius)
		{
			return false;
		}
	}
	else if (!RaySphere(startPtA, rayDir, posB, shapeA.radius + shapeB.radius, t0, t1))
	{
		return false;
	}

	// Change from [0, 1] to [0, dt];
	t0 *= dt;
	t1 *= dt;

	// If the collision in only in the past, there will be no future collision for this frame
	if (t1 < 0) return false;

	// Get earliest positive time of impact
	timeOfImpact = t0 < 0.0f ? 0.0f : t0;

	// If the earliest collision is too far in the future, then there's no collision this frame
	if (timeOfImpact > dt) {
		return false;
	}

	// Get the points on the respective points of collision and return true
	Vec3 newPosA = posA + velA * timeOfImpact;
	Vec3 newPosB = posB + velB * timeOfImpact;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * shapeA.radius;
	ptOnB = newPosB - ab * shapeB.radius;
	return true;
}

bool Intersections::SphereSphereStatic(const ShapeSphere* sphereA, const ShapeSphere* sphereB, const Vec3& posA, const Vec3& posB, Vec3& ptOnA, Vec3& ptOnB) {
	const Vec3 ab = posB - posA;
	Vec3 norm = ab;
	norm.Normalize();

	ptOnA = posA + norm * sphereA->radius;
	ptOnB = posB - norm * sphereB->radius;

	const float radiusAB = sphereA->radius + sphereB->radius;
	const float lengthSquare = ab.GetLengthSqr();
	if (lengthSquare <= (radiusAB * radiusAB)) {
		return true;
	}

	return false;
}


bool Intersections::Intersect(Body* bodyA, Body* bodyB, Contact& contact) {
	contact.a = bodyA;
	contact.b = bodyB;
	contact.timeOfImpact = 0.0f;

	if (bodyA->shape->GetType() == Shape::ShapeType::SHAPE_SPHERE && bodyB->shape->GetType() == Shape::ShapeType::SHAPE_SPHERE) {
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->shape;

		Vec3 posA = bodyA->position;
		Vec3 posB = bodyB->position;

		if (SphereSphereStatic(sphereA, sphereB, posA, posB, contact.ptOnAWorldSpace, contact.ptOnBWorldSpace)) {
			contact.normal = posA - posB;
			contact.normal.Normalize();

			contact.ptOnALocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
			contact.ptOnBLocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

			Vec3 ab = bodyB->position - bodyA->position;
			float r = ab.GetMagnitude() - (sphereA->radius + sphereB->radius);
			contact.separationDistance = r;
			return true;
		}
	}
	else {
		Vec3 ptOnA;
		Vec3 ptOnB;
		const float bias = 0.001f;
		if (GJK_DoesIntersect(bodyA, bodyB, bias, ptOnA, ptOnB)) {
			// There was an intersection, so get the contact data
			Vec3 normal = ptOnB - ptOnA;
			normal.Normalize();

			ptOnA -= normal * bias;
			ptOnB += normal * bias;

			contact.normal = normal;

			contact.ptOnAWorldSpace = ptOnA;
			contact.ptOnBWorldSpace = ptOnB;

			contact.ptOnALocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
			contact.ptOnBLocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

			Vec3 ab = bodyB->position - bodyA->position;
			float r = (ptOnA - ptOnB).GetMagnitude();
			contact.separationDistance = -r;
			return true;
		}

		// There was no collision, but we still want the contact data, so get it
		GJK_ClosestPoints(bodyA, bodyB, ptOnA, ptOnB);
		contact.ptOnAWorldSpace = ptOnA;
		contact.ptOnBWorldSpace = ptOnB;

		contact.ptOnALocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
		contact.ptOnBLocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

		Vec3 ab = bodyB->position - bodyA->position;
		float r = (ptOnA - ptOnB).GetMagnitude();
		contact.separationDistance = r;
	}
	return false;
}

bool Intersections::ConservativeAdvance(Body& bodyA, Body& bodyB, float dt, Contact& contact)
{
	contact.a = &bodyA;
	contact.b = &bodyB;

	float toi = 0.0f;

	int numIters = 0;

	// Advance the positions of the bodies until they touch or there's not time left
	while (dt > 0.0f) {
		// Check for intersection
		bool didIntersect = Intersect(&bodyA, &bodyB, contact);
		if (didIntersect) {
			contact.timeOfImpact = toi;
			bodyA.Update(-toi);
			bodyB.Update(-toi);
			return true;
		}

		++numIters;
		if (numIters > 10) {
			break;
		}

		// Get the vector from the closest point on A to the closest point on B
		Vec3 ab = contact.ptOnBWorldSpace - contact.ptOnAWorldSpace;
		ab.Normalize();

		// project the relative velocity onto the ray of shortest distance
		Vec3 relativeVelocity = bodyA.linearVelocity - bodyB.linearVelocity;
		float orthoSpeed = relativeVelocity.Dot(ab);

		// Add to the orthoSpeed the maximum angular speeds of the relative shapes
		float angularSpeedA = bodyA.shape->FastestLinearSpeed(bodyA.angularVelocity, ab);
		float angularSpeedB = bodyB.shape->FastestLinearSpeed(bodyB.angularVelocity, ab * -1.0f);
		orthoSpeed += angularSpeedA + angularSpeedB;
		if (orthoSpeed <= 0.0f) {
			break;
		}

		float timeToGo = contact.separationDistance / orthoSpeed;
		if (timeToGo > dt) {
			break;
		}

		dt -= timeToGo;
		toi += timeToGo;
		bodyA.Update(timeToGo);
		bodyB.Update(timeToGo);
	}

	// Unwind the clock
	bodyA.Update(-toi);
	bodyB.Update(-toi);
	return false;
}