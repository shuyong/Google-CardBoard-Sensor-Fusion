
#include <math.h>

#ifndef VECTOR3D_H
#define VECTOR3D_H

class Vector3d {
	public: double x;
	public: double y;
	public: double z;

	public: Vector3d() {
		this->x = 0.0D;
		this->y = 0.0D;
		this->z = 0.0D;
	}

	public: Vector3d(const double xx, const double yy, const double zz) {
		set(xx, yy, zz);
	}

	public: void set(const double xx, const double yy, const double zz) {
		this->x = xx;
		this->y = yy;
		this->z = zz;
	}

	public: void setComponent(const int i, const double val) {
		if (i == 0) {
			this->x = val;
		} else if (i == 1) {
			this->y = val;
		} else {
			this->z = val;
		}
	}

	public: void setZero() {
		this->x = 0.0D;
		this->y = 0.0D;
		this->z = 0.0D;
	}

	public: void set(const Vector3d& other) {
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
	}

	public: void scale(const double s) {
		this->x *= s;
		this->y *= s;
		this->z *= s;
	}

	public: void normalize() {
		const double d = length();
		if (d != 0.0D) {
			scale(1.0D / d);
		}
	}

	// 矢量点乘：一个矢量在另外一个矢量上的投影长度。
	// a . b = |a||b|cos(θ)
	// 两个单位矢量的夹角: cos(θ)
	// cos(θ) > 0 : 两个矢量同向
	// cos(θ) = 0 : 两个矢量垂直
	// cos(θ) < 0 : 两个矢量反向
	public: static double dot(const Vector3d& a, const Vector3d& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	// 矢量的模长
	public: double length() {
		return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
	}

	public: bool sameValues(const Vector3d& other) {
		return (this->x == other.x) && (this->y == other.y) && (this->z == other.z);
	}

	// 矢量加: c = a + b。
	// 矢量a, b构成一个以c为对角线的平行四边形。
	public: static void add(const Vector3d& a, const Vector3d& b, Vector3d& result) {
		result.set(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	// 矢量差: c = a - b。
	// 矢量a, b, c构成一个封闭三角形。
	public: static void sub(const Vector3d& a, const Vector3d& b, Vector3d& result) {
		result.set(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	// 矢量叉乘: a * b = c
	// |c| = |a * b| = |a||b|sin(θ)
	// 矢量c的方向垂直于矢量a,b的共面，采用右手定则确定。
	public: static void cross(const Vector3d& a, const Vector3d& b, Vector3d& result) {
		result.set(a.y * b.z - a.z * b.y,
			   a.z * b.x - a.x * b.z,
			   a.x * b.y - a.y * b.x);
	}

	// 计算矢量v的正交矢量。
	public: static void ortho(const Vector3d& v, Vector3d& result) {
		int k = largestAbsComponent(v) - 1;
		if (k < 0) {
			k = 2;
		}
		result.setZero();
		result.setComponent(k, 1.0D);

		cross(v, result, result);
		result.normalize();
	}

	public: static int largestAbsComponent(const Vector3d& v) {
		const double xAbs = fabs(v.x);
		const double yAbs = fabs(v.y);
		const double zAbs = fabs(v.z);
		if (xAbs > yAbs) {
			if (xAbs > zAbs) {
				return 0;
			}
			return 2;
		}
		if (yAbs > zAbs) {
			return 1;
		}
		return 2;
	}
};

#endif // VECTOR3D_H

