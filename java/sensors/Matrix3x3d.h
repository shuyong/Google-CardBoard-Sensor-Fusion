#include "Vector3d.h"

#ifndef MATRIX3X3D_H
#define MATRIX3X3D_H

class Matrix3x3d {
	public: double m[9];

	public: Matrix3x3d() {
		this->m[0] = 0.0D;
		this->m[1] = 0.0D;
		this->m[2] = 0.0D;
		this->m[3] = 0.0D;
		this->m[4] = 0.0D;
		this->m[5] = 0.0D;
		this->m[6] = 0.0D;
		this->m[7] = 0.0D;
		this->m[8] = 0.0D;
	}

	public: Matrix3x3d(const double m00, const double m01, const double m02,
			   const double m10, const double m11, const double m12,
			   const double m20, const double m21, const double m22) {
		this->m[0] = m00;
		this->m[1] = m01;
		this->m[2] = m02;
		this->m[3] = m10;
		this->m[4] = m11;
		this->m[5] = m12;
		this->m[6] = m20;
		this->m[7] = m21;
		this->m[8] = m22;
	}

	public: Matrix3x3d(const Matrix3x3d& o) {
		this->m[0] = o.m[0];
		this->m[1] = o.m[1];
		this->m[2] = o.m[2];
		this->m[3] = o.m[3];
		this->m[4] = o.m[4];
		this->m[5] = o.m[5];
		this->m[6] = o.m[6];
		this->m[7] = o.m[7];
		this->m[8] = o.m[8];
	}

	public: void set(const double m00, const double m01, const double m02,
			 const double m10, const double m11, const double m12,
			 const double m20, const double m21, const double m22) {
		this->m[0] = m00;
		this->m[1] = m01;
		this->m[2] = m02;
		this->m[3] = m10;
		this->m[4] = m11;
		this->m[5] = m12;
		this->m[6] = m20;
		this->m[7] = m21;
		this->m[8] = m22;
	}

	public: void set(const Matrix3x3d& o) {
		this->m[0] = o.m[0];
		this->m[1] = o.m[1];
		this->m[2] = o.m[2];
		this->m[3] = o.m[3];
		this->m[4] = o.m[4];
		this->m[5] = o.m[5];
		this->m[6] = o.m[6];
		this->m[7] = o.m[7];
		this->m[8] = o.m[8];
	}

	public: void setZero() {
		this->m[0] = 0.0D;
		this->m[1] = 0.0D;
		this->m[2] = 0.0D;
		this->m[3] = 0.0D;
		this->m[4] = 0.0D;
		this->m[5] = 0.0D;
		this->m[6] = 0.0D;
		this->m[7] = 0.0D;
		this->m[8] = 0.0D;
	}

	public: void setIdentity() {
		this->m[0] = 1.0D; this->m[1] = 0.0D; this->m[2] = 0.0D;
		this->m[3] = 0.0D; this->m[4] = 1.0D; this->m[5] = 0.0D;
		this->m[6] = 0.0D; this->m[7] = 0.0D; this->m[8] = 1.0D;
	}

	// 设置矩阵对角线元素
	public: void setSameDiagonal(const double d) {
		this->m[0] = d;
		this->m[4] = d;
		this->m[8] = d;
	}

	public: double get(const int row, const int col) {
		return this->m[3 * row + col];
	}

	public: void set(const int row, const int col, const double value) {
		this->m[3 * row + col] = value;
	}

	// 取列元素为矢量
	public: void getColumn(const int col, Vector3d& v) {
		v.x = this->m[col];
		v.y = this->m[col + 3];
		v.z = this->m[col + 6];
	}

	// 将矢量设为列元素
	public: void setColumn(const int col, const Vector3d& v) {
		this->m[col]     = v.x;
		this->m[col + 3] = v.y;
		this->m[col + 6] = v.z;
	}

	public: void scale(const double s) {
		for (int i = 0; i < 9; i++) {
			this->m[i] *= s;
		}
	}

	// +=
	public: void plusEquals(const Matrix3x3d& b) {
		for (int i = 0; i < 9; i++) {
			this->m[i] += b.m[i];
		}
	}

	// -=
	public: void minusEquals(const Matrix3x3d& b) {
		for (int i = 0; i < 9; i++) {
			this->m[i] -= b.m[i];
		}
	}

	// 自身转置
	public: void transpose() {
		double tmp = this->m[1];
		this->m[1] = this->m[3];
		this->m[3] = tmp;

		tmp = this->m[2];
		this->m[2] = this->m[6];
		this->m[6] = tmp;

		tmp = this->m[5];
		this->m[5] = this->m[7];
		this->m[7] = tmp;
	}

	// 转置赋值
	public: void transpose(Matrix3x3d& result) {
        	const double m1 = this->m[1];
        	const double m2 = this->m[2];
        	const double m5 = this->m[5];
		result.m[0] = this->m[0];
		result.m[1] = this->m[3];
		result.m[2] = this->m[6];
		result.m[3] = m1;
		result.m[4] = this->m[4];
		result.m[5] = this->m[7];
		result.m[6] = m2;
		result.m[7] = m5;
		result.m[8] = this->m[8];
	}

	public: static void add(const Matrix3x3d& a, const Matrix3x3d& b, Matrix3x3d& result) {
		result.m[0] = a.m[0] + b.m[0],
		result.m[1] = a.m[1] + b.m[1],
		result.m[2] = a.m[2] + b.m[2],
		result.m[3] = a.m[3] + b.m[3],
		result.m[4] = a.m[4] + b.m[4],
		result.m[5] = a.m[5] + b.m[5],
		result.m[6] = a.m[6] + b.m[6],
		result.m[7] = a.m[7] + b.m[7],
		result.m[8] = a.m[8] + b.m[8];
	}

	// 张量积(tensor product): 矩阵 ⊗ 矩阵
	public: static void mult(const Matrix3x3d& a, const Matrix3x3d& b, Matrix3x3d& result) {
		result.set(a.m[0] * b.m[0] + a.m[1] * b.m[3] + a.m[2] * b.m[6],
			   a.m[0] * b.m[1] + a.m[1] * b.m[4] + a.m[2] * b.m[7],
			   a.m[0] * b.m[2] + a.m[1] * b.m[5] + a.m[2] * b.m[8],
			   a.m[3] * b.m[0] + a.m[4] * b.m[3] + a.m[5] * b.m[6],
			   a.m[3] * b.m[1] + a.m[4] * b.m[4] + a.m[5] * b.m[7],
			   a.m[3] * b.m[2] + a.m[4] * b.m[5] + a.m[5] * b.m[8],
			   a.m[6] * b.m[0] + a.m[7] * b.m[3] + a.m[8] * b.m[6],
			   a.m[6] * b.m[1] + a.m[7] * b.m[4] + a.m[8] * b.m[7],
			   a.m[6] * b.m[2] + a.m[7] * b.m[5] + a.m[8] * b.m[8]);
	}

	// 张量积(tensor product): 矩阵 ⊗ 矢量
	// [ rx ]   [ a0 a1 a2 ]   [ vx ]
	// [ ry ] = [ a3 a4 a5 ] ⊗ [ vy ]
	// [ rz ]   [ a6 a7 a8 ]   [ vz ]
	public: static void mult(const Matrix3x3d& a, const Vector3d& v, Vector3d& result) {
		const double x = a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z;
		const double y = a.m[3] * v.x + a.m[4] * v.y + a.m[5] * v.z;
		const double z = a.m[6] * v.x + a.m[7] * v.y + a.m[8] * v.z;
		result.x = x;
		result.y = y;
		result.z = z;
	}

	// 行列式
	public: double determinant() {
		return get(0, 0) * (get(1, 1) * get(2, 2) - get(2, 1) * get(1, 2))
		     - get(0, 1) * (get(1, 0) * get(2, 2) - get(1, 2) * get(2, 0))
		     + get(0, 2) * (get(1, 0) * get(2, 1) - get(1, 1) * get(2, 0));
	}

	// Matrix^{-1}
	public: bool invert(Matrix3x3d& result) {
		const double d = determinant();
		if (d == 0.0D) {
			return false;
		}
		const double invdet = 1.0D / d;

		result.set((this->m[4] * this->m[8] - this->m[7] * this->m[5]) * invdet,
			  -(this->m[1] * this->m[8] - this->m[2] * this->m[7]) * invdet,
			   (this->m[1] * this->m[5] - this->m[2] * this->m[4]) * invdet,
			  -(this->m[3] * this->m[8] - this->m[5] * this->m[6]) * invdet,
			   (this->m[0] * this->m[8] - this->m[2] * this->m[6]) * invdet,
			  -(this->m[0] * this->m[5] - this->m[3] * this->m[2]) * invdet,
			   (this->m[3] * this->m[7] - this->m[6] * this->m[4]) * invdet,
			  -(this->m[0] * this->m[7] - this->m[6] * this->m[1]) * invdet,
			   (this->m[0] * this->m[4] - this->m[3] * this->m[1]) * invdet);

		return true;
	}
};

#endif // MATRIX3X3D_H
