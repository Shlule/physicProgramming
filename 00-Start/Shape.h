#pragma once
#include "code/Math/Matrix.h"
#include "code/Math/Bounds.h"
#include "code/Math/Quat.h"

class Shape {
public:
	enum class ShapeType
	{
		SHAPE_SPHERE
	};

	virtual ShapeType GetType() const = 0;
	virtual Mat3 InertialTensor() const = 0;
	virtual Vec3 GetCenterOfMass() const { return centerOfMass; }
	

protected:
	Vec3 centerOfMass;
};


class ShapeSphere : public Shape {
public:
	ShapeSphere(float radiusP) : radius(radiusP)
	{
		centerOfMass.Zero();
	}

	
	ShapeType GetType() const override { return ShapeType::SHAPE_SPHERE; }
	Mat3 InertialTensor() const override;
	float radius;
};

