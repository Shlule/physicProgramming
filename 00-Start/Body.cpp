#include "Body.h"
#include "Shape.h"

Vec3 Body::GetCenterOfMassWorldSpace() const {
	const Vec3 centerOfMass = shape->GetCenterOfMass();
	const Vec3 pos = position + orientation.RotatePoint(centerOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassBodySpace() const {
	return shape->GetCenterOfMass();
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPoint) {
	const Vec3 tmp = worldPoint - GetCenterOfMassWorldSpace();
	const Quat invertOrient = orientation.Inverse();
	Vec3  bodySpace = invertOrient.RotatePoint(tmp);
	return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& bodyPoint) {
	Vec3 worldSpace = GetCenterOfMassWorldSpace()
		+ orientation.RotatePoint(bodyPoint);
	return worldSpace;
}

void Body::ApplyImpulseLinear(const Vec3& impulse) {
	if (inverseMass == 0.0f) return;
	linearVelocity += impulse * inverseMass;
}

Mat3 Body::GetInverseInertialTensorBodySpace() const {
	Mat3 inertialTensor = shape->InertialTensor();
	Mat3 inverseInertialTensor = inertialTensor.Inverse() * inverseMass;
	return inverseInertialTensor;
}

Mat3 Body::GetInverseInertialTensorWorldSpace() const {
	Mat3 inertialTensor = shape->InertialTensor();
	Mat3 inverseInertialTensor = inertialTensor.Inverse() * inverseMass;
	Mat3 orient = orientation.ToMat3();

	inverseInertialTensor = orient * inverseInertialTensor * orient.Transpose();
	return inverseInertialTensor;
}

void Body::ApplyImpulseAngular(const Vec3& impulse) {
	if (inverseMass == 0.0f) return;

	// L = I w = r x p
// dL = I dw = r x J
// dw = I^-1 * ( r x J )
	angularVelocity += GetInverseInertialTensorWorldSpace() * impulse;
	const float maxAngularSpeed = 30.0f;
	if (angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed) {
		angularVelocity.Normalize();
		angularVelocity *= maxAngularSpeed;
	}
}