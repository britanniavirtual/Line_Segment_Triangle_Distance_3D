#pragma once

class Vector3D
{
public:

	double x, y, z;

	Vector3D(double x, double y, double z)//Constructor
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3D()//Overloaded Constructor
	{
		x = 0;
		y = 0;
		z = 0;
	}

	//Add two vectors
	Vector3D operator+(const Vector3D& inputVector) const
	{
		return Vector3D(x + inputVector.x, y + inputVector.y, z + inputVector.z);
	}

	//Subtract two vectors
	Vector3D operator-(const Vector3D& inputVector) const
	{
		return Vector3D(x - inputVector.x, y - inputVector.y, z - inputVector.z);
	}

	//Multiply two vectors
	Vector3D operator*(const Vector3D& inputVector) const
	{
		return Vector3D(x * inputVector.x, y * inputVector.y, z * inputVector.z);
	}

	//Divide two vectors
	Vector3D operator/(const Vector3D& inputVector) const
	{
		return Vector3D(x / inputVector.x, y / inputVector.y, z / inputVector.z);
	}

	//Add floating point to vector
	Vector3D operator+(double f) const
	{
		return Vector3D(f + x, f + y, f + z);
	}

	//Add floating point to vector
	Vector3D operator+(float f) const
	{
		return Vector3D(f + x, f + y, f + z);
	}

	//Subtract floating point from vector
	Vector3D operator-(double f) const
	{
		return Vector3D(f - x, f - y, f - z);
	}

	//Multiply vector by floating point
	Vector3D operator*(double f) const
	{
		return Vector3D(f*x, f*y, f*z);
	}

	//Divide vector by floating point
	Vector3D operator/(double f) const
	{
		return Vector3D(f / x, f / y, f / z);
	}


	void operator+=(const Vector3D& inputVector)
	{
		this->x = inputVector.x + this->x;
		this->y = inputVector.y + this->y;
		this->z = inputVector.z + this->z;
	}

	void operator-=(const Vector3D& inputVector)
	{
		this->x -= inputVector.x + this->x;
		this->y -= inputVector.y + this->y;
		this->z -= inputVector.z + this->z;
	}

	void operator*=(const Vector3D& inputVector)
	{
		this->x = inputVector.x * this->x;
		this->y = inputVector.y * this->y;
		this->z = inputVector.z * this->z;
	}

	void operator/=(const Vector3D& inputVector)
	{
		this->x = this->x / inputVector.x;
		this->y = this->y / inputVector.y;
		this->z = this->z / inputVector.z;
	}

	void operator*=(double f)
	{
		this->x = f * this->x;
		this->y = f * this->y;
		this->z = f * this->z;
	}

	void operator=(const float inputFloat)
	{
		this->x = inputFloat;
		this->y = inputFloat;
		this->z = inputFloat;
	}

	void operator/=(double f)
	{
		this->x = this->x / f;
		this->y = this->y / f;
		this->z = this->z / f;
	}
};

class VectorUtils
{
public:

	float dist(Vector3D a, Vector3D b);
	Vector3D scalarTimesVector(float scalar, const Vector3D v);
	bool isPointInsideTriangle3D(Vector3D p, Vector3D a, Vector3D b, Vector3D c);
	Vector3D closestPointTriangle(Vector3D p, Vector3D a, Vector3D b, Vector3D c);
	Vector3D closestPointOnSegment(Vector3D linePointA, Vector3D linePointB, Vector3D point);
	Vector3D closestPointOnPlaneToPoint(Vector3D planeNormal, Vector3D planePoint, Vector3D point);
	Vector3D triangleToNormal(Vector3D a, Vector3D b, Vector3D c);
	Vector3D cross(const Vector3D& A, const Vector3D& B);

	void closestDistanceBetweenLineSegmentAndTriangle3D(Vector3D p1, Vector3D p2, Vector3D v1, Vector3D v2, Vector3D v3, Vector3D * nearestPoint, float * nearestDist);
};

