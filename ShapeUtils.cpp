#include "ShapeUtils.h"

int FindPointFurthestInDir(const Vec3* pts, const int num, const Vec3& dir)
{
	int maxIndex = 0;
	float maxDist = dir.Dot(pts[0]);
	for (int i = 1; i < num; i++) {
		float dist = dir.Dot(pts[i]);
		if (dist > maxDist) {
			maxDist = dist;
			maxIndex = i;
		}
	}
	return maxIndex;
}

float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pt)
{
	Vec3 ab = b - a;
	ab.Normalize();

	Vec3 ray = pt - a;
	Vec3 projectionOnAB = ab * ray.Dot(ab);
	Vec3 perpindicular = ray - projectionOnAB;
	return perpindicular.GetMagnitude();
}

Vec3 FindPointFurthestFromLine(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB)
{
	int maxIdx = 0;
	float maxDist = DistanceFromLine(ptA, ptB, pts[0]);
	for (int i = 1; i < num; i++) {
		float dist = DistanceFromLine(ptA, ptB, pts[i]);
		if (dist > maxDist) {
			maxDist = dist;
			maxIdx = i;
		}
	}
	return pts[maxIdx];
}

float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt)
{
	Vec3 ab = b - a;
	Vec3 ac = c - a;
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	Vec3 ray = pt - a;
	float dist = ray.Dot(normal);
	return dist;
}

Vec3 FindPointFurthestFromTriangle(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB, const Vec3& ptC)
{
	int maxIdx = 0;
	float maxDist = DistanceFromTriangle(ptA, ptB, ptC, pts[0]);
	for (int i = 1; i < num; i++) {
		float dist = DistanceFromTriangle(ptA, ptB, ptC, pts[i]);
		if (dist * dist > maxDist * maxDist) {
			maxDist = dist;
			maxIdx = i;
		}
	}
	return pts[maxIdx];
}

void BuildTetrahedron(const Vec3* verts, const int num, std::vector<Vec3>& hullPts, std::vector<Tri>& hullTris) {
	hullPts.clear();
	hullTris.clear();

	Vec3 points[4];

	int idx = FindPointFurthestInDir(verts, num, Vec3(1, 0, 0));
	points[0] = verts[idx];
	idx = FindPointFurthestInDir(verts, num, points[0] * -1.0f);
	points[1] = verts[idx];
	points[2] = FindPointFurthestFromLine(verts, num, points[0], points[1]);
	points[3] = FindPointFurthestFromTriangle(verts, num, points[0], points[1], points[2]);

	// This is important for making sure the ordering is CCW for all faces.
	float dist = DistanceFromTriangle(points[0], points[1], points[2], points[3]);
	if (dist > 0.0f) {
		std::swap(points[0], points[1]);
	}

	// Build the tetrahedron
	hullPts.push_back(points[0]);
	hullPts.push_back(points[1]);
	hullPts.push_back(points[2]);
	hullPts.push_back(points[3]);

	hullTris.push_back(Tri{ 0, 1, 2 });
	hullTris.push_back(Tri{ 0, 2, 3 });
	hullTris.push_back(Tri{ 2, 1, 3 });
	hullTris.push_back(Tri{ 1, 0, 3 });
}

void ExpandConvexHull(std::vector<Vec3>& hullPoints, std::vector<Tri>& hullTris, const std::vector<Vec3>& verts)
{
	std::vector< Vec3 > externalVerts = verts;
	RemoveInternalPoints(hullPoints, hullTris, externalVerts);

	while (externalVerts.size() > 0) {
		int ptIdx = FindPointFurthestInDir(externalVerts.data(), (int)externalVerts.size(), externalVerts[0]);

		Vec3 pt = externalVerts[ptIdx];

		// Remove this element
		externalVerts.erase(externalVerts.begin() + ptIdx);

		AddPoint(hullPoints, hullTris, pt);

		RemoveInternalPoints(hullPoints, hullTris, externalVerts);
	}

	RemoveUnreferencedVerts(hullPoints, hullTris);
}

void RemoveInternalPoints(const std::vector<Vec3>& hullPoints, const std::vector<Tri>& hullTris, std::vector<Vec3>& checkPts)
{
	for (int i = 0; i < checkPts.size(); i++) {
		const Vec3& pt = checkPts[i];

		bool isExternal = false;
		for (int t = 0; t < hullTris.size(); t++) {
			const Tri& tri = hullTris[t];
			const Vec3& a = hullPoints[tri.a];
			const Vec3& b = hullPoints[tri.b];
			const Vec3& c = hullPoints[tri.c];

			// If the point is in front of any triangle then it's external
			float dist = DistanceFromTriangle(a, b, c, pt);
			if (dist > 0.0f) {
				isExternal = true;
				break;
			}
		}

		// If it's not external, then it's inside the polyhedron and should be removed
		if (!isExternal) {
			checkPts.erase(checkPts.begin() + i);
			i--;
		}
	}

	// Also remove any points that are just a little too close to the hull points
	for (int i = 0; i < checkPts.size(); i++) {
		const Vec3& pt = checkPts[i];

		bool isTooClose = false;
		for (int j = 0; j < hullPoints.size(); j++) {
			Vec3 hullPt = hullPoints[j];
			Vec3 ray = hullPt - pt;
			// 1cm is too close
			if (ray.GetLengthSqr() < 0.01f * 0.01f) {
				isTooClose = true;
				break;
			}
		}

		if (isTooClose) {
			checkPts.erase(checkPts.begin() + i);
			i--;
		}
	}
}

bool IsEdgeUnique(const std::vector<Tri>& tris, const std::vector<int>& facingTris, const int ignoreTri, const Edge& edge)
{
	for (int i = 0; i < facingTris.size(); i++) {
		const int triIdx = facingTris[i];
		if (ignoreTri == triIdx) {
			continue;
		}

		const Tri& tri = tris[triIdx];

		Edge edges[3];
		edges[0].a = tri.a;
		edges[0].b = tri.b;

		edges[1].a = tri.b;
		edges[1].b = tri.c;

		edges[2].a = tri.c;
		edges[2].b = tri.a;

		for (int e = 0; e < 3; e++) {
			if (edge == edges[e]) {
				return false;
			}
		}
	}
	return true;
}

void AddPoint(std::vector<Vec3>& hullPoints, std::vector<Tri>& hullTris, const Vec3& pt)
{
	// This point is outside
	// Now we need to remove old triangles and build new ones

	// Find all the triangles that face this point
	std::vector< int > facingTris;
	for (int i = (int)hullTris.size() - 1; i >= 0; i--) {
		const Tri& tri = hullTris[i];

		const Vec3& a = hullPoints[tri.a];
		const Vec3& b = hullPoints[tri.b];
		const Vec3& c = hullPoints[tri.c];

		const float dist = DistanceFromTriangle(a, b, c, pt);
		if (dist > 0.0f) {
			facingTris.push_back(i);
		}
	}

	// Now find all edges that are unique to the tris, these will be the edges that form the new triangles
	std::vector<Edge> uniqueEdges;
	for (int i = 0; i < facingTris.size(); i++) {
		const int triIdx = facingTris[i];
		const Tri& tri = hullTris[triIdx];

		Edge edges[3];
		edges[0].a = tri.a;
		edges[0].b = tri.b;

		edges[1].a = tri.b;
		edges[1].b = tri.c;

		edges[2].a = tri.c;
		edges[2].b = tri.a;

		for (int e = 0; e < 3; e++) {
			if (IsEdgeUnique(hullTris, facingTris, triIdx, edges[e])) {
				uniqueEdges.push_back(edges[e]);
			}
		}
	}

	// Now remove the old facing tris
	for (int i = 0; i < facingTris.size(); i++) {
		hullTris.erase(hullTris.begin() + facingTris[i]);
	}

	// Now add the new point
	hullPoints.push_back(pt);
	const int newPtIdx = (int)hullPoints.size() - 1;

	// Now add triangles for each unique edge
	for (int i = 0; i < uniqueEdges.size(); i++) {
		const Edge& edge = uniqueEdges[i];

		Tri tri;
		tri.a = edge.a;
		tri.b = edge.b;
		tri.c = newPtIdx;
		hullTris.push_back(tri);
	}
}

void RemoveUnreferencedVerts(std::vector<Vec3>& hullPoints, std::vector<Tri>& hullTris)
{
	for (int i = 0; i < hullPoints.size(); i++) {

		bool isUsed = false;
		for (int j = 0; j < hullTris.size(); j++) {
			const Tri& tri = hullTris[j];

			if (tri.a == i || tri.b == i || tri.c == i) {
				isUsed = true;
				break;
			}
		}

		if (isUsed) {
			continue;
		}

		for (int j = 0; j < hullTris.size(); j++) {
			Tri& tri = hullTris[j];
			if (tri.a > i) {
				tri.a--;
			}
			if (tri.b > i) {
				tri.b--;
			}
			if (tri.c > i) {
				tri.c--;
			}
		}

		hullPoints.erase(hullPoints.begin() + i);
		i--;
	}
}

void BuildConvexHull(
	const std::vector<Vec3>& verts,
	std::vector<Vec3>& hullPts,
	std::vector<Tri>& hullTris
) {
	if (verts.size() < 4) {
		return;
	}

	// Build a tetrahedron
	BuildTetrahedron(verts.data(), (int)verts.size(), hullPts, hullTris);

	ExpandConvexHull(hullPts, hullTris, verts);
}

bool IsExternal(const std::vector< Vec3 >& pts, const std::vector<Tri>& tris, const Vec3& pt)
{
	bool isExternal = false;
	for (int t = 0; t < tris.size(); t++) {
		const Tri& tri = tris[t];
		const Vec3& a = pts[tri.a];
		const Vec3& b = pts[tri.b];
		const Vec3& c = pts[tri.c];

		// If the point is in front of any triangle then it's external
		float dist = DistanceFromTriangle(a, b, c, pt);
		if (dist > 0.0f) {
			isExternal = true;
			break;
		}
	}

	return isExternal;
}