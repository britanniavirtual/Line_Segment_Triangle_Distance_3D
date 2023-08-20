#include "pch.h"
#include "UniHeader.h"

int main()
{
	//Test data

	//3D triangle
	Vector3D v1(0, 0, 0);
	Vector3D v2(0, 0, 1);
	Vector3D v3(1, 0, 0);

	//Line segment
	Vector3D linePos1(0.2, 1, 0.2);
	Vector3D linePos2(1, 2, 0);

	//Outputs
	Vector3D nearestPoint;
	float nearestDist;

	vecUtils.closestDistanceBetweenLineSegmentAndTriangle3D(linePos1, linePos2, v1, v2, v3, &nearestPoint, &nearestDist);

	{
		cout << ":" << nearestPoint.x << " " << nearestPoint.y << " " << nearestPoint.z << endl;
		cout << "dist " << nearestDist << endl;
	}

	system("PAUSE");
}
