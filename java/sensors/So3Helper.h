#include <math.h>

#include "Vector3d.h"
#include "Matrix3x3d.h"

#ifndef SO3HELPER_H
#define SO3HELPER_H

// The 3D rotation group, often denoted SO(3)
// 计算旋转的辅助函数
// 动态版本。
// 在多线程应用中使用。
class So3Helper {
#ifndef M_SQRT1_2
	// sqrt(1/2)
	private: static constexpr double M_SQRT1_2 = 0.7071067811865475D;
#endif
	// 1 / 6
	private: static constexpr double ONE_6TH  = 0.1666666666666667D;
	// 1 / 24
	private: static constexpr double ONE_24TH = 0.0416666666666667D;
	private: Vector3d temp;
	private: Vector3d sO3FromTwoVecN; // 矢量n = a * b
	private: Vector3d sO3FromTwoVecA; // 矢量a
	private: Vector3d sO3FromTwoVecB; // 矢量b
	private: Vector3d sO3FromTwoVecRotationAxis;	// 旋转轴
	private: Matrix3x3d sO3FromTwoVec33R1;
	private: Matrix3x3d sO3FromTwoVec33R2;
	private: Vector3d muFromSO3R2;
	private: Vector3d rotationPiAboutAxisTemp; // 为rotationPiAboutAxis函数准备的临时变量。

	public: So3Helper() {
	}

	// 根据旋转前后的矢量a, b求出旋转矩阵
	public: void sO3FromTwoVec(const Vector3d& a, const Vector3d& b, Matrix3x3d& result) {
		// 矢量sO3FromTwoVecN垂直于矢量a, b共面。
		Vector3d::cross(a, b, sO3FromTwoVecN);
		// 当矢量sO3FromTwoVecN的模长为0. 
		// 矢量叉乘: a * b = c
		// |c| = |a * b| = |a||b|sin(θ)
		// 当θ = 180°, 模长为0.
		if (sO3FromTwoVecN.length() == 0.0D) {
			// 矢量点乘：两个单位矢量的夹角: cos(θ)
			const double dot = Vector3d::dot(a, b);
			// cos(θ) > 0 : 两个矢量同向
			// cos(θ) = 0 : 两个矢量垂直
			// cos(θ) < 0 : 两个矢量反向
			if (dot >= 0.0D) {
				// 两个矢量同向，旋转矩阵为单位矩阵。
				result.setIdentity();
			} else {
				// 两个矢量反向，则θ = 180°。
				// 矢量sO3FromTwoVecRotationAxis(旋转轴)垂直于矢量a
				Vector3d::ortho(a, sO3FromTwoVecRotationAxis);
				// 绕矢量sO3FromTwoVecRotationAxis(旋转轴)旋转180°。
				rotationPiAboutAxis(sO3FromTwoVecRotationAxis, result);
				// 罗格里格参数(Rodrigues' Parameters)，为避免在π处的奇异值，绕特定的轴旋转180°。
			}
			return;
		}
		sO3FromTwoVecA.set(a);
		sO3FromTwoVecB.set(b);

		sO3FromTwoVecN.normalize();
		sO3FromTwoVecA.normalize();
		sO3FromTwoVecB.normalize();

		Matrix3x3d& r1 = sO3FromTwoVec33R1;
		r1.setColumn(0, sO3FromTwoVecA);
		r1.setColumn(1, sO3FromTwoVecN);
		Vector3d::cross(sO3FromTwoVecN, sO3FromTwoVecA, temp);
		r1.setColumn(2, temp);

		Matrix3x3d& r2 = sO3FromTwoVec33R2;
		r2.setColumn(0, sO3FromTwoVecB);
		r2.setColumn(1, sO3FromTwoVecN);
		Vector3d::cross(sO3FromTwoVecN, sO3FromTwoVecB, temp);
		r2.setColumn(2, temp);

		r1.transpose();
		// result = r2 * r1'
		Matrix3x3d::mult(r2, r1, result);
	}

	private: void rotationPiAboutAxis(const Vector3d& v, Matrix3x3d& result) {
		const double theta = M_PI;
		const double invTheta = 1 / M_PI;
		rotationPiAboutAxisTemp.set(v);
		rotationPiAboutAxisTemp.scale(theta / rotationPiAboutAxisTemp.length());

		// kA = sin(theta) * invTheta;                       // sin(M_PI) = 0;
		const double kA = 0.0D;
		// kB = (1.0D - cos(theta)) * (invTheta * invTheta); // cos(M_PI) = -1;
		const double kB = 2.0D * (invTheta * invTheta) ;
		rodriguesSo3Exp(rotationPiAboutAxisTemp, kA, kB, result);
	}

	// 从微分角度矢量ω，求出微分旋转矩阵。
	// Process nonlinear vector function, f().
	// ω : The rotation vector is assumed to be in axis angle form. Control input, u.
	// Angle = ||ω|| and Axis = ω / ||ω||, with ω = [ω_x, ω_y, ω_z]^T.
	public: void sO3FromMu(const Vector3d& w, Matrix3x3d& result) {
		const double thetaSq = Vector3d::dot(w, w);
		const double theta = sqrt(thetaSq);
		double kA;
		double kB;
		if (thetaSq < 1.0E-8D) {
			kA = 1.0D - thetaSq * ONE_6TH;
			kB = 0.5D;
		}
		else if (thetaSq < 1.0E-6D) {
			kB = 0.5D - thetaSq * ONE_24TH;
			kA = 1.0D - thetaSq * ONE_6TH * (1.0D - thetaSq * ONE_6TH);
		}
		else {
			const double invTheta = 1.0D / theta;
			kA = sin(theta) * invTheta;
			kB = (1.0D - cos(theta)) * (invTheta * invTheta);
		}
		rodriguesSo3Exp(w, kA, kB, result);
	}

	// 从微分旋转矩阵，求出微分角度矢量。
	// ω : The rotation vector is assumed to be in axis angle form.
	// Angle = ||ω|| and Axis = ω / ||ω||, with ω = [ω_x, ω_y, ω_z]^T.
	public: void muFromSO3(Matrix3x3d& so3, Vector3d& result) {
		const double cosAngle = (so3.get(0, 0) + so3.get(1, 1) + so3.get(2, 2) - 1.0D) * 0.5D;
		result.set((so3.get(2, 1) - so3.get(1, 2)) / 2.0D,
			   (so3.get(0, 2) - so3.get(2, 0)) / 2.0D,
			   (so3.get(1, 0) - so3.get(0, 1)) / 2.0D);

		const double sinAngleAbs = result.length();
		if (cosAngle > M_SQRT1_2) {
			if (sinAngleAbs > 0.0D) {
				result.scale(asin(sinAngleAbs) / sinAngleAbs);
			}
		} else if (cosAngle > -M_SQRT1_2) {
			const double angle = acos(cosAngle);
			result.scale(angle / sinAngleAbs);
		} else {
			const double angle = M_PI - asin(sinAngleAbs);
			const double d0 = so3.get(0, 0) - cosAngle;
			const double d1 = so3.get(1, 1) - cosAngle;
			const double d2 = so3.get(2, 2) - cosAngle;

			Vector3d& r2 = muFromSO3R2;
			if ((d0 * d0 > d1 * d1) && (d0 * d0 > d2 * d2)) {
				r2.set( d0,
				       (so3.get(1, 0) + so3.get(0, 1)) / 2.0D,
				       (so3.get(0, 2) + so3.get(2, 0)) / 2.0D);
			} else if (d1 * d1 > d2 * d2) {
				r2.set((so3.get(1, 0) + so3.get(0, 1)) / 2.0D,
					d1,
				       (so3.get(2, 1) + so3.get(1, 2)) / 2.0D);
			} else {
				r2.set((so3.get(0, 2) + so3.get(2, 0)) / 2.0D,
				       (so3.get(2, 1) + so3.get(1, 2)) / 2.0D,
				        d2);
			}
			if (Vector3d::dot(r2, result) < 0.0D) {
				r2.scale(-1.0D);
			}
			r2.normalize();
			r2.scale(angle);
			result.set(r2);
		}
	}

	// 罗格里格旋转公式(Rodrigues' rotation formula)
	// 罗格里格参数(Rodrigues' Parameters)
	private: static void rodriguesSo3Exp(const Vector3d& w, const double kA, const double kB, Matrix3x3d& result) {
		/* 
		 * w : angle vector, θ = ||w||
		 * kA = sin(θ) / θ;
		 * kB = (1 - cos(θ)) / (θ * θ);
		 *
		 *     0                            1                            2 
		 * 0 | 1.0 - kB * (w.y^2 + w.z^2) , kB * (w.x * w.y) - kA * w.z, kB * (w.x * w.z) + kA * w.y, |
		 * 1 | kB * (w.x * w.y) + kA * w.z, 1.0 - kB * (w.x^2 + w.z^2) , kB * (w.y * w.z) - kA * w.x, |
		 * 2 | kB * (w.x * w.z) - kA * w.y, kB * (w.y * w.z) + kA * w.x, 1.0 - kB * (w.x^2 + w.y^2) , |
		 */
		const double wx2 = w.x * w.x;
		const double wy2 = w.y * w.y;
		const double wz2 = w.z * w.z;

		result.set(0, 0, 1.0D - kB * (wy2 + wz2));
		result.set(1, 1, 1.0D - kB * (wx2 + wz2));
		result.set(2, 2, 1.0D - kB * (wx2 + wy2));
		
		double a;
		double b;
		
		a = kA * w.z;
		b = kB * (w.x * w.y);
		result.set(0, 1, b - a);
		result.set(1, 0, b + a);

		a = kA * w.y;
		b = kB * (w.x * w.z);
		result.set(0, 2, b + a);
		result.set(2, 0, b - a);

		a = kA * w.x;
		b = kB * (w.y * w.z);
		result.set(1, 2, b - a);
		result.set(2, 1, b + a);
	}
};

#endif // SO3HELPER_H
