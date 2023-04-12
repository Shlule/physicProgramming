#pragma once
#include "code/Math/Vector.h"
#include "code/Renderer/model.h"
#include "code/Math/Quat.h"

class Body
{
public:
	Vec3 position;
	Quat orientation;
	Vec3 linearVelocity;
	Vec3 angularVelocity;
	float friction;
	float inverseMass;
	float elasticity;
	Shape* shape;

	void ApplyImpulseLinear(const Vec3& impulse);
	void ApplyImpulseAngular(const Vec3& impulse);

	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassBodySpace() const;

	Vec3 WorldSpaceToBodySpace(const Vec3& worldPoint);
	Vec3 BodySpaceToWorldSpace(const Vec3& bodyPoint);


	Mat3 GetInverseInertialTensorBodySpace() const;
	Mat3 GetInverseInertialTensorWorldSpace() const;

	void applyImpulse(const Vec3& impulsePoint, const Vec3& impulse);

	void Update(const float dt_sec);

};


