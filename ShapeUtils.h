#pragma once
#include "code/Math/Vector.h"
#include <vector>

struct Tri {
	int a;
	int b;
	int c;
};

struct Edge {
	int a;
	int b;

	bool operator == (const Edge& rhs) const {
		return ((a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a));
	}
};

int FindPointFurthestInDir(const Vec3* pts, const int num, const Vec3& dir);

float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pt);

Vec3 FindPointFurthestFromLine(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB);

float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt);

Vec3 FindPointFurthestFromTriangle(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB, const Vec3& ptC);

void BuildTetrahedron(const Vec3* verts, const int num, std::vector<Vec3>& hullPts, std::vector<Tri>& hullTris);

void RemoveInternalPoints(const std::vector<Vec3>& hullPoints, const std::vector<Tri>& hullTris, std::vector<Vec3>& checkPts);

bool IsEdgeUnique(const std::vector<Tri>& tris, const std::vector<int>& facingTris, const int ignoreTri, const Edge& edge);

void AddPoint(std::vector<Vec3>& hullPoints, std::vector<Tri>& hullTris, const Vec3& pt);

void RemoveUnreferencedVerts(std::vector<Vec3>& hullPoints, std::vector<Tri>& hullTris);

void ExpandConvexHull(std::vector<Vec3>& hullPoints, std::vector<Tri>& hullTris, const std::vector<Vec3>& verts);

bool IsExternal(const std::vector<Vec3>& pts, const std::vector<Tri>& tris, const Vec3& pt);

void BuildConvexHull(
	const std::vector<Vec3>& verts,
	std::vector<Vec3>& hullPts,
	std::vector<Tri>& hullTris
);