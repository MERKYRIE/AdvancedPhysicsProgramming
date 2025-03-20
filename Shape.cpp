#include "Shape.h"
#include "code/Math/Matrix.h"

Mat3 ShapeSphere::InertiaTensor() const
{
	Mat3 tensor;
	tensor.Zero();
	tensor.rows[0][0] = 2.0f * radius * radius / 5.0f;
	tensor.rows[1][1] = 2.0f * radius * radius / 5.0f;
	tensor.rows[2][2] = 2.0f * radius * radius / 5.0f;
	return tensor;
}

Bounds ShapeSphere::GetBounds(const Vec3& pos, const Quat& orient) const
{
	Bounds tmp;
	tmp.mins = Vec3(-radius) + pos;
	tmp.maxs = Vec3(radius) + pos;
	return tmp;
}

Bounds ShapeSphere::GetBounds() const
{
	Bounds tmp;
	tmp.mins = Vec3(-radius);
	tmp.maxs = Vec3(radius);
	return tmp;
}

Vec3 ShapeSphere::Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias)
{
	return pos + dir * (radius + bias);
}

void ShapeBox::Build(const Vec3* pts, const int num)
{
	for (int i = 0; i < num; ++i)
	{
		bounds.Expand(points[i]);
	}

	points.clear();
	points.push_back(Vec3{ bounds.mins.x, bounds.mins.y, bounds.mins.z });
	points.push_back(Vec3{ bounds.maxs .x, bounds.mins.y, bounds.mins.z });
	points.push_back(Vec3{ bounds.mins.x, bounds.maxs.y, bounds.mins.z });
	points.push_back(Vec3{ bounds.mins.x, bounds.mins.y, bounds.maxs.z });

	points.push_back(Vec3{ bounds.maxs.x, bounds.maxs.y, bounds.maxs.z });
	points.push_back(Vec3{ bounds.mins.x, bounds.maxs.y, bounds.maxs.z });
	points.push_back(Vec3{ bounds.maxs.x, bounds.mins.y, bounds.maxs.z });
	points.push_back(Vec3{ bounds.maxs.x, bounds.maxs.y, bounds.mins.z });

	centerOfMass = (bounds.maxs + bounds.mins) * 0.5f;
}

