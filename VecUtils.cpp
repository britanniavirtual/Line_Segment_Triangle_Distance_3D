#include "pch.h"
#include "UniHeader.h"

using namespace std;

//Implementation of max min
template <typename T>
const T& my_max(const T& a, const T& b) {
	return (a > b) ? a : b;
}

template <typename T>
const T& my_min(const T& a, const T& b) {
	return (a < b) ? a : b;
}

float dot(const Vector3D& a, const Vector3D& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float lengthSquared(const Vector3D& v) {
	return dot(v, v);
}

template <typename T>
T my_clamp(const T& value, const T& minVal, const T& maxVal)
{
	return my_max(minVal, my_min(value, maxVal));
}

template <typename T>
T my_abs(T value)
{
	return(value < 0) ? -value : value;
}

Vector3D VectorUtils::closestPointTriangle(Vector3D p, Vector3D a, Vector3D b, Vector3D c)
{
	const Vector3D ab = b - a;
	const Vector3D ac = c - a;
	const Vector3D ap = p - a;

	const float d1 = dot(ab, ap);
	const float d2 = dot(ac, ap);
	if (d1 <= 0.f && d2 <= 0.f) return a; //#1

	const Vector3D bp = p - b;
	const float d3 = dot(ab, bp);
	const float d4 = dot(ac, bp);
	if (d3 >= 0.f && d4 <= d3) return b; //#2

	const Vector3D cp = p - c;
	const float d5 = dot(ab, cp);
	const float d6 = dot(ac, cp);
	if (d6 >= 0.f && d5 <= d6) return c; //#3

	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
	{
		const float v = d1 / (d1 - d3);
		return a + ab * v; //#4
	}

	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
	{
		const float v = d2 / (d2 - d6);
		return a + ac * v; //#5
	}

	const float va = d3 * d6 - d5 * d4;
	if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
	{
		const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * v; //#6
	}

	const float denom = 1.f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;

	return a + ab * v + ac * w; //#0
}

float VectorUtils::dist(Vector3D a, Vector3D b)
{
	return(sqrt(pow((float)(b.x - a.x), (float)2) + pow((float)(b.y - a.y), (float)2) + pow((float)(b.z - a.z), (float)2)));
}


Vector3D VectorUtils::scalarTimesVector(float scalar, const Vector3D v)
{
	return Vector3D(v.x * scalar, v.y * scalar, v.z * scalar);
}



bool VectorUtils::isPointInsideTriangle3D(Vector3D p, Vector3D a, Vector3D b, Vector3D c)
{
	Vector3D v0 = c - a;
	Vector3D v1 = b - a;
	Vector3D v2 = p - a;

	float dot00 = v0.x * v0.x + v0.y * v0.y + v0.z * v0.z;
	float dot01 = v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
	float dot02 = v0.x * v2.x + v0.y * v2.y + v0.z * v2.z;
	float dot11 = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
	float dot12 = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

	float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	return (u >= 0) && (v >= 0) && (u + v <= 1);
}


Vector3D scalarDivideByVector(float scalar, const Vector3D v)
{
	return Vector3D(v.x * scalar, v.y * scalar, v.z * scalar);
}


Vector3D VectorUtils::closestPointOnSegment(Vector3D linePointA, Vector3D linePointB, Vector3D point)
{
	Vector3D lineDirection = linePointB - linePointA;
	Vector3D pointToLineStart = linePointA - point;

	float t = -(pointToLineStart.x * lineDirection.x + pointToLineStart.y * lineDirection.y + pointToLineStart.z * lineDirection.z) /
		(lineDirection.x * lineDirection.x + lineDirection.y * lineDirection.y + lineDirection.z * lineDirection.z);

	t = my_max(0.0f, my_min(1.0f, t)); // Ensure t is clamped between 0 and 1

	return linePointA + scalarTimesVector(t, lineDirection);// t * lineDirection;
}


Vector3D VectorUtils::closestPointOnPlaneToPoint(Vector3D planeNormal, Vector3D planePoint, Vector3D point)
{
	Vector3D pointToPlane = point - planePoint;
	float distance = dot(pointToPlane, planeNormal);

	Vector3D closestPoint = point - scalarTimesVector(distance, planeNormal);

	return closestPoint;
}



Vector3D subtract2(const Vector3D& v1, const Vector3D& v2) {
	return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

double dotProduct(const Vector3D& v1, const Vector3D& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3D crossProduct2(const Vector3D& v1, const Vector3D& v2) {
	return {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

double magnitude2(const Vector3D& v) {
	return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}



Vector3D intersectionPoint(Vector3D lineStart, Vector3D lineEnd, Vector3D planePoint, Vector3D planeNormal)
{
	Vector3D lineDir = lineEnd - lineStart;
	double denom = dotProduct(lineDir, planeNormal);

	if (std::abs(denom) < 1e-6) {
		// Line is parallel to the plane
		return { INFINITE, INFINITE, INFINITE }; // Or some other value indicating no intersection
	}

	double t = dotProduct(planePoint - lineStart, planeNormal) / denom;

	if (t < 0.0 || t > 1.0) {
		// Intersection point is outside the line segment
		return { INFINITE, INFINITE, INFINITE }; // Or some other value indicating no intersection
	}

	Vector3D intersection;
	intersection.x = lineStart.x + t * lineDir.x;
	intersection.y = lineStart.y + t * lineDir.y;
	intersection.z = lineStart.z + t * lineDir.z;

	return intersection;
}






void VectorUtils::closestDistanceBetweenLineSegmentAndTriangle3D(Vector3D p1, Vector3D p2, Vector3D v1, Vector3D v2, Vector3D v3, Vector3D * nearestPoint, float * nearestDist)
{

	//////////////////////////////////////////////////////////
	//DESC:
	//Case 1: Line intersects triangle plane --> Get nearest of the 3 possible points compute distance.
		//Case 1b: If inside the triangle distance is: 0.

	//Case 2: Line does not intersect plane
		//For A and B
		//Get nearest point distance to: triangle itself, triangles edges. Nearest dist is nearest dist result.
	
	//Distances in question
	//////////////////////////////////////////////////////////

	Vector3D finalPoint(0, 0, 0);
	Vector3D pointPlane(0, 0, 0);
	Vector3D point_01_Edge(0, 0, 0);
	Vector3D point_01_Triangle(0, 0, 0);
	Vector3D point_02_Edge(0, 0, 0);
	Vector3D point_02_Triangle(0, 0, 0);

	//Distances:
	float finalDistance = INFINITE;

	float distance_Plane;

	float distance_01_NearestEdge;
	float distance_01_TriangleArea;
	
	float distance_02_NearestEdge;
	float distance_02_TriangleArea;

	//[(1/2) Plane intersect and ONLY intersect]
	Vector3D planeIntersectedEdgePoint;
	Vector3D ip = intersectionPoint(p2, p1, Vector3D(0, 0, 0), Vector3D(0, 1, 0));

	if (isPointInsideTriangle3D(ip, v1, v2, v3))//Line goes through the triangle
	{
		finalPoint = ip;
		finalDistance = 0.0;
	}
	else
	{
		bool intersectsPlane = false;

		if (ip.x == INFINITE)
		{
			//"No intersect.";
		}
		else
		{
			intersectsPlane = true;
			//"intersect.";
		}

		{
			Vector3D c1 = closestPointOnSegment(v1, v2, ip);
			Vector3D c2 = closestPointOnSegment(v2, v3, ip);
			Vector3D c3 = closestPointOnSegment(v3, v1, ip);

			float d1 = dist(c1, ip);
			float d2 = dist(c2, ip);
			float d3 = dist(c3, ip);

			int n = 0;
			//Closest edge to plane intersect point
			if (d1 <= d2 && d1 <= d3)
			{
				n = 1;
				planeIntersectedEdgePoint = c1;
			}

			if (d2 <= d3 && d2 <= d1)
			{
				n = 2;
				planeIntersectedEdgePoint = c2;
			}

			if (d3 <= d2 && d3 <= d1)
			{
				n = 3;
				planeIntersectedEdgePoint = c3;
			}

			distance_Plane = dist(planeIntersectedEdgePoint, ip);//Distance if plane intersects
			pointPlane = planeIntersectedEdgePoint;
		}

		//2/2) Nearest positions

		//NODE 1
		//1/2) Triangle edges
		Vector3D p1NearestEdgePoint;
		{
			Vector3D c1 = closestPointOnSegment(v1, v2, p1);
			Vector3D c2 = closestPointOnSegment(v2, v3, p1);
			Vector3D c3 = closestPointOnSegment(v3, v1, p1);

			float d1 = dist(c1, p1);
			float d2 = dist(c2, p1);
			float d3 = dist(c3, p1);

			int n = 0;
			//Closest edge to plane intersect point
			if (d1 <= d2 && d1 <= d3)
			{
				n = 1;
				p1NearestEdgePoint = c1;
			}

			if (d2 <= d3 && d2 <= d1)
			{
				n = 2;
				p1NearestEdgePoint = c2;
			}

			if (d3 <= d2 && d3 <= d1)
			{
				n = 3;
				p1NearestEdgePoint = c3;
			}

			distance_01_NearestEdge = dist(p1NearestEdgePoint, p1);
			point_01_Edge = p1NearestEdgePoint;
		}

		//2/2) Triangle area
		Vector3D closestTrianglePoint_01 = closestPointTriangle(p1, v1, v2, v3);
		distance_01_TriangleArea = dist(closestTrianglePoint_01, p1);
		point_01_Triangle = closestTrianglePoint_01;

		//NODE 2
		//1/2) Triangle edges
		Vector3D p2NearestEdgePoint;
		{
			Vector3D c1 = closestPointOnSegment(v1, v2, p2);
			Vector3D c2 = closestPointOnSegment(v2, v3, p2);
			Vector3D c3 = closestPointOnSegment(v3, v1, p2);

			float d1 = dist(c1, p2);
			float d2 = dist(c2, p2);
			float d3 = dist(c3, p2);

			int n = 0;
			//Closest edge to plane intersect point
			if (d1 <= d2 && d1 <= d3)
			{
				n = 1;
				p2NearestEdgePoint = c1;
			}

			if (d2 <= d3 && d2 <= d1)
			{
				n = 2;
				p2NearestEdgePoint = c2;
			}

			if (d3 <= d2 && d3 <= d1)
			{
				n = 3;
				p2NearestEdgePoint = c3;
			}

			distance_02_NearestEdge = dist(p2NearestEdgePoint, p2);
			point_02_Edge = p2NearestEdgePoint;
		}

		//2/2) Triangle area
		Vector3D closestTrianglePoint_02 = closestPointTriangle(p2, v1, v2, v3);
		distance_02_TriangleArea = dist(closestTrianglePoint_01, p2);
		point_02_Triangle = closestTrianglePoint_01;


		//[Find closest distance]
		if (distance_Plane <= distance_01_NearestEdge)
		{
			if (distance_Plane <= distance_01_TriangleArea)
			{
				if (distance_Plane <= distance_02_NearestEdge)
				{
					if (distance_Plane <= distance_02_TriangleArea)
					{
						finalDistance = distance_Plane;
						finalPoint = pointPlane;
					}
				}
			}
		}

		if (distance_01_NearestEdge <= distance_Plane)
		{
			if (distance_01_NearestEdge <= distance_01_TriangleArea)
			{
				if (distance_01_NearestEdge <= distance_02_NearestEdge)
				{
					if (distance_01_NearestEdge <= distance_02_TriangleArea)
					{
						finalDistance = distance_01_NearestEdge;
						finalPoint = point_01_Edge;
					}
				}
			}
		}

		if (distance_01_TriangleArea <= distance_Plane)
		{
			if (distance_01_TriangleArea <= distance_01_NearestEdge)
			{
				if (distance_01_TriangleArea <= distance_02_NearestEdge)
				{
					if (distance_01_TriangleArea <= distance_02_TriangleArea)
					{
						finalDistance = distance_01_TriangleArea;
						finalPoint = point_01_Triangle;
					}
				}
			}
		}

		if (distance_02_NearestEdge <= distance_Plane)
		{
			if (distance_02_NearestEdge <= distance_01_NearestEdge)
			{
				if (distance_02_NearestEdge <= distance_01_TriangleArea)
				{
					if (distance_02_NearestEdge <= distance_02_TriangleArea)
					{
						finalDistance = distance_02_NearestEdge;
						finalPoint = point_02_Edge;
					}
				}
			}
		}

		if (distance_02_TriangleArea <= distance_Plane)
		{
			if (distance_02_TriangleArea <= distance_01_NearestEdge)
			{
				if (distance_02_TriangleArea <= distance_01_TriangleArea)
				{
					if (distance_02_TriangleArea <= distance_02_NearestEdge)
					{
						finalDistance = distance_02_TriangleArea;
						finalPoint = point_02_Triangle;
					}
				}
			}
		}

	}

	*nearestDist = finalDistance;
	*nearestPoint = finalPoint;
}