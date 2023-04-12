#include"Shape.h"
#include"code/Math/Matrix.h"

Mat3 ShapeSphere::InertialTensor() const {
	Mat3 tensor;
	tensor.rows[0][0] = 2.0f * radius * radius / 5.0f;
	tensor.rows[1][1] = 2.0f * radius * radius / 5.0f;
	tensor.rows[2][2] = 2.0f * radius * radius / 5.0f;
}