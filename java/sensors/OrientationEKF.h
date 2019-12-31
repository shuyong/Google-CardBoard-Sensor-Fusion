
#include <math.h>

#include "Vector3d.h"
#include "Matrix3x3d.h"
#include "So3Helper.h"
#include "So3Util.h"

#ifndef ORIENTATIONEKF_H
#define ORIENTATIONEKF_H

#ifndef min
#define min(a,b) (a)<(b)?(a):(b)
#endif

// 误差状态旋转矩阵
class OrientationEKF {
	private: static constexpr float NS2S = 1.0E-9F;
	private: static constexpr double MIN_ACCEL_NOISE_SIGMA = 0.75D;
	private: static constexpr double MAX_ACCEL_NOISE_SIGMA = 7.0D;
	private: double rotationMatrix[16]; // OpenGL 4x4 rotation matrix
	private: Matrix3x3d so3SensorFromWorld; // 姿态旋转矩阵，body frame。
	private: Matrix3x3d so3LastMotion;      // 微分旋转矩阵。初始化为 I。F : the Jacobian matrix of partial derivatives of f() with respect to x.
	private: Matrix3x3d mP;		// 估计误差协方差矩阵: P = F P F' + Q。Covariance of Kalman filter state (P in common formulation).
	private: Matrix3x3d mQ;		// 过程噪声协方差矩阵。Covariance of the process noise (Q in common formulation).
	private: Matrix3x3d mR;		// 测量噪声协方差矩阵，磁力。
	private: Matrix3x3d mRaccel;	// 测量噪声协方差矩阵，加速度。Covariance of the accelerometer measurement (R in common formulation).
	private: Matrix3x3d mS;		// 创新协方差(Covariance of innovation)，系统不确定性: S = H P H' + R。Covariance of innovation (S in common formulation).
	private: Matrix3x3d mH;		// 测量 Jacobian 矩阵。Jacobian of the measurements (H in common formulation).
	private: Matrix3x3d mK;  	// 卡尔曼增益。Gain of the Kalman filter (K in common formulation).
	private: Vector3d mNu;   // 创新矢量。从预测姿态旋转矩阵，先验得到的创新矢量。Parameter update a.k.a. innovation vector. (\nu in common formulation).
	private: Vector3d mz;    // 测量矢量。当前测量得到加速度或磁场矢量。Measurement vector (z in common formulation).
	private: Vector3d mh;    // 测量函数。先验的指向地心矢量(加速度)，或指向水平磁极矢量(磁场强度)。Current prediction vector (g in common formulation).
	private: Vector3d mu;    // 当前测量得到微分角度矢量。Control input, currently this is only the gyroscope data (\mu in common formulation).
	private: Vector3d mx;    // 状态矢量(代表旋转误差的旋转矢量)。后验得到 mx = mK * mNu。Update of the state vector. (x in common formulation).
	private: Vector3d down;  // 参考矢量，重力矢量，指向地心。
	private: Vector3d north; // 参考矢量，磁场矢量，指向磁极。
	private: long sensorTimeStampGyro;
	private: Vector3d lastGyro; // 上次测量得到微分角度矢量 mu。
	private: double previousAccelNorm = 0.0D;		// 上一次加速度矢量范数
	private: double movingAverageAccelNormChange = 0.0D;	// 运动时加速度矢量范数变化数值
	private: float filteredGyroTimestep;
	private: bool timestepFilterInit = false;
	private: int numGyroTimestepSamples;
	private: bool gyroFilterValid = true;
	private: Matrix3x3d getPredictedGLMatrixTempM1;
	private: Matrix3x3d getPredictedGLMatrixTempM2;
	private: Vector3d getPredictedGLMatrixTempV1;
	private: Matrix3x3d setHeadingDegreesTempM1;
	private: Matrix3x3d processGyroTempM1;
	private: Matrix3x3d processGyroTempM2;
	private: Matrix3x3d processAccTempM1;
	private: Matrix3x3d processAccTempM2;
	private: Matrix3x3d processAccTempM3;
	private: Matrix3x3d processAccTempM4;
	private: Matrix3x3d processAccTempM5;
	private: Vector3d processAccTempV1;
	private: Vector3d processAccTempV2;
	private: Vector3d processAccVDelta; // 加速度矢量的测量值与估计值的差值。
	private: Vector3d processMagTempV1;
	private: Vector3d processMagTempV2;
	private: Vector3d processMagTempV3;
	private: Vector3d processMagTempV4;
	private: Vector3d processMagTempV5;
	private: Matrix3x3d processMagTempM1;
	private: Matrix3x3d processMagTempM2;
	private: Matrix3x3d processMagTempM3;
	private: Matrix3x3d processMagTempM4;
	private: Matrix3x3d processMagTempM5;
	private: Matrix3x3d updateCovariancesAfterMotionTempM1;
	private: Matrix3x3d updateCovariancesAfterMotionTempM2;
	private: Matrix3x3d accObservationFunctionForNumericalJacobianTempM;
	private: Matrix3x3d magObservationFunctionForNumericalJacobianTempM;
	private: bool alignedToGravity;	// 是否进行过重力校正
	private: bool alignedToNorth;	// 是否进行过磁力校正
	private: So3Helper so3Helper;	// 进行SO(3)计算的辅助函数

	public: OrientationEKF() {
		reset();
	}

	public: void reset() {
		this->sensorTimeStampGyro = 0L;

		this->so3SensorFromWorld.setIdentity();
		this->so3LastMotion.setIdentity();

		// 对 P 的初值不信任
		const double initialSigmaP = 5.0D;
		this->mP.setZero();
		this->mP.setSameDiagonal(initialSigmaP * initialSigmaP);

		// 对 Q 最信任。后面根据 dT 进行缩放。100Hz -> 1E-4
		const double initialSigmaQ = 1.0D;
		this->mQ.setZero();
		this->mQ.setSameDiagonal(initialSigmaQ * initialSigmaQ);

		// 对 Mag R 比较信任。
		const double initialSigmaR = 0.25D;
		this->mR.setZero();
		this->mR.setSameDiagonal(initialSigmaR * initialSigmaR);

		// 对 Accel R 比较信任。后面根据 Accel 的测量矢量模长动态调整。
		const double initialSigmaRaccel = MIN_ACCEL_NOISE_SIGMA;
		this->mRaccel.setZero();
		this->mRaccel.setSameDiagonal(initialSigmaRaccel * initialSigmaRaccel);

		this->mS.setZero();
		this->mH.setZero();
		this->mK.setZero();
		this->mNu.setZero();
		this->mz.setZero();
		this->mh.setZero();
		this->mu.setZero();
		this->mx.setZero();

		// 机体坐标系 (body frame) : Android(ENU)
		// 所有加速度传感器本身都是“加速度正”。
		// 在零横滚角和零俯仰角的参考位置，加速度计测量值为 +1g，
		// 因为 z 轴和与重力相等的加速度都对齐并指向上。
		this->down.set (0.0D, 0.0D, 9.81D); // Acceleration reference vector, m/s^2
		this->north.set(0.0D, 1.0D, 0.00D); // Magnetic field reference vector

		this->alignedToGravity = false;
		this->alignedToNorth = false;
	}

	public: bool isReady() {
		return this->alignedToGravity;
	}

	public: double getHeadingDegrees() {
		const double x = this->so3SensorFromWorld.get(2, 0);
		const double y = this->so3SensorFromWorld.get(2, 1);
		const double mag = sqrt(x * x + y * y);
		if (mag < 0.1D) {
			return 0.0D;
		}
		double heading = -90.0D - atan2(y, x) / M_PI * 180.0D;
		if (heading < 0.0D) {
			heading += 360.0D;
		}
		if (heading >= 360.0D) {
			heading -= 360.0D;
		}

		return heading;
	}

	public: void setHeadingDegrees(const double heading) {
		const double currentHeading = getHeadingDegrees();
		const double deltaHeading = heading - currentHeading;
		const double s = sin(deltaHeading / 180.0D * M_PI);
		const double c = cos(deltaHeading / 180.0D * M_PI);

		const double deltaHeadingRotationVals[3][3] = {
			{ c,   -s,    0.0D },
			{ s,    c,    0.0D },
			{ 0.0D, 0.0D, 1.0D } };

		arrayAssign(deltaHeadingRotationVals, this->setHeadingDegreesTempM1);
		Matrix3x3d::mult(this->so3SensorFromWorld, this->setHeadingDegreesTempM1,
				this->so3SensorFromWorld);
	}

	public: double* getGLMatrix() {
		return glMatrixFromSo3(this->so3SensorFromWorld);
	}

	public: double* getPredictedGLMatrix(double secondsAfterLastGyroEvent) {
		Vector3d& pmu = this->getPredictedGLMatrixTempV1;
		pmu.set(this->lastGyro);		// mu : control input / function
		pmu.scale(-secondsAfterLastGyroEvent);	// Angular velocity * dT = Angle
		Matrix3x3d& so3PredictedMotion = this->getPredictedGLMatrixTempM1;
		// 从微分角度矢量 mu，求出微分旋转矩阵。
		// sO3FromMu : Process nonlinear vector function, f().
		So3Util::sO3FromMu(pmu, so3PredictedMotion);

		Matrix3x3d& so3PredictedState = this->getPredictedGLMatrixTempM2;
		Matrix3x3d::mult(so3PredictedMotion, this->so3SensorFromWorld,
				so3PredictedState);

		return glMatrixFromSo3(so3PredictedState);
	}

	public: Matrix3x3d& getRotationMatrix() {
		return this->so3SensorFromWorld;
	}

	public: static void arrayAssign(const double data[3][3], Matrix3x3d& m) {
		m.set(data[0][0], data[0][1], data[0][2],
		      data[1][0], data[1][1], data[1][2],
		      data[2][0], data[2][1], data[2][2]);
	}

	public: bool isAlignedToGravity() {
		return this->alignedToGravity;
	}

	public: bool isAlignedToNorth() {
		return this->alignedToNorth;
	}

	// The gyroscope data is assumed to be in axis angle form.
	// Angle = ||v|| and Axis = v / ||v||, with v = [v_x, v_y, v_z]^T.
	public: void processGyro(const Vector3d& gyro, const long sensorTimeStamp) {
		float kTimeThreshold = 0.04F; // 25HZ
		float kdTdefault     = 0.01F; //100HZ
		if (this->sensorTimeStampGyro != 0L) {
			float dT = (float) (sensorTimeStamp - this->sensorTimeStampGyro) * NS2S;
			if (dT > kTimeThreshold) {
				dT = this->gyroFilterValid ? this->filteredGyroTimestep : kdTdefault;
			}
			else {
				filterGyroTimestep(dT);
			}
			// 微分角度矢量 mu 为轴-角形式，旋转角度用矢量的模长表示。
			// Since the gyroscope value is a start from sensor 
			// transformation we need to invert it to have a sensor
			// from start transformation, hence the minus sign.
			// For more info:
			// http://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-gyro
			this->mu.set(gyro);	// mu : control input / function
			this->mu.scale((double)(-dT));	// Angular velocity * dT = Angle
			// 从微分角度矢量 mu，求出微分旋转矩阵。
			// sO3FromMu : Process nonlinear vector function, f().
			// so3LastMotion 同时也是 F matrix
			So3Util::sO3FromMu(this->mu, this->so3LastMotion);

			// 最新的微分旋转矩阵 x 原先的姿态旋转矩阵 = 当前的姿态旋转矩阵
			Matrix3x3d::mult(this->so3LastMotion, this->so3SensorFromWorld,
					this->so3SensorFromWorld);

			// 运动后更新协方差矩阵 P = F P F'
			updateCovariancesAfterMotion();

			// P = P + dT * Q * dT 
			this->processGyroTempM2.set(this->mQ);
			this->processGyroTempM2.scale((double)(dT * dT));	// 根据 dT 缩放，时间越小越可信。
			this->mP.plusEquals(this->processGyroTempM2);
			// P = F P F' + dT * Q * dT
		}
		this->sensorTimeStampGyro = sensorTimeStamp;
		this->lastGyro.set(gyro);
	}

	private: void updateAccelCovariance(const double currentAccelNorm) {
        	const double currentAccelNormChange = abs(currentAccelNorm - this->previousAccelNorm);
        	this->previousAccelNorm = currentAccelNorm;
        	const double kSmoothingFactor = 0.5D;
        	this->movingAverageAccelNormChange = kSmoothingFactor * currentAccelNormChange
                	+ kSmoothingFactor * this->movingAverageAccelNormChange;
        	const double kMaxAccelNormChange = 0.15D;
        	const double kMinAccelNoiseSigma = MIN_ACCEL_NOISE_SIGMA;
        	const double kMaxAccelNoiseSigma = MAX_ACCEL_NOISE_SIGMA;
        	const double normChangeRatio = this->movingAverageAccelNormChange / kMaxAccelNormChange;
        	const double accelNoiseSigma = min(kMaxAccelNoiseSigma,
                	kMinAccelNoiseSigma + normChangeRatio * (kMaxAccelNoiseSigma - kMinAccelNoiseSigma));
        	this->mRaccel.setSameDiagonal(accelNoiseSigma * accelNoiseSigma);
	}

	// 在向量微积分中，Jacobian 矩阵是一阶偏导数以一定方式排列成的矩阵。
	// Jacobian 矩阵的重要性在于它体现了一个可微方程与给出点的最优线性逼近。
	//
	// Numerical Jacobian: 
	// http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture7.pdf
	// 3.7: Jacobian: Numerical Computation
	//
	// Rotation Matrix   : M
	// innovation vector : ν
	// 
	// J_ij = (f_i(M + Δt) - f_i(M + ε)) / ε
	// f_i(M + Δt) = ν
	// f_i(M + ε)  = Δ
	//
	// let ε = 1.0E-7
	//
	// i axis: V(ε,0,0) * M -> vector Δi
	// j axis: V(0,ε,0) * M -> vector Δj
	// k axis: V(0,0,ε) * M -> vector Δk
	//
	//        i axis      j axis      k axis     
	//      [ ν_x - Δi_x, ν_x - Δj_x, ν_x - Δk_x ]
	// JF = [ ν_y - Δi_y, ν_y - Δj_y, ν_y - Δk_y ] / ε
	//      [ ν_z - Δi_z, ν_z - Δj_z, ν_z - Δk_z ]
	//
	public: void processAcc(const Vector3d& acc, const long sensorTimeStamp) {
		// 测量加速度矢量 mz，包含指向地心的重力矢量。
		this->mz.set(acc);
		// 根据 Accel 的测量矢量模长动态调整加速度测量噪声协方差矩阵
		updateAccelCovariance(this->mz.length());
		if (this->alignedToGravity) {
			// acc: 为了 Jacobian 数值矩阵的观测函数。为求 H matrix。
			// 观测函数 h() 计算机体坐标系中的参考矢量的预测姿态，
			// 并与测量矢量 z 计算差异得到创新矢量 ν。
			accObservationFunctionForNumericalJacobian(this->so3SensorFromWorld,
					this->mNu);

			const double eps = 1.0E-7D;
			// 按照列(坐标轴)更新测量矩阵 H, Jacobian matrix
			for (int dof = 0; dof < 3; dof++) {
				Vector3d& delta = this->processAccVDelta;
				delta.setZero();
				delta.setComponent(dof, eps);

				// 从微分角度矢量 ε，求出微分旋转矩阵。
				// sO3FromMu : Process nonlinear vector function, f().
				So3Util::sO3FromMu(delta, this->processAccTempM1);
				// 给当前的姿态旋转矩阵的一个坐标轴一个扰动 ε。
				Matrix3x3d::mult(this->processAccTempM1, this->so3SensorFromWorld,
						this->processAccTempM2);

				accObservationFunctionForNumericalJacobian(
						this->processAccTempM2, this->processAccTempV1);

				Vector3d& withDelta = this->processAccTempV1;

				Vector3d::sub(this->mNu, withDelta, this->processAccTempV2);
				this->processAccTempV2.scale(1.0D / eps);
				this->mH.setColumn(dof, this->processAccTempV2);
			}
			// M3 = mH'
			this->mH.transpose(this->processAccTempM3);
			// M4 = mP * mH'
			Matrix3x3d::mult(this->mP, this->processAccTempM3,
					this->processAccTempM4);
			// M5 = mH * mP * mH'
			Matrix3x3d::mult(this->mH, this->processAccTempM4,
					this->processAccTempM5);
			// mS = mH * mP * mH' + mRaccel
			Matrix3x3d::add(this->processAccTempM5, this->mRaccel, this->mS);

			// M3 = mS^-1
			this->mS.invert(this->processAccTempM3);
			// M4 = mH'
			this->mH.transpose(this->processAccTempM4);
			// M5 = mH' * mS^-1
			Matrix3x3d::mult(this->processAccTempM4, this->processAccTempM3,
					this->processAccTempM5);
			// mK = mP * mH' * mS^-1
			Matrix3x3d::mult(this->mP, this->processAccTempM5, this->mK);

			// 状态矢量 mx (代表旋转误差的旋转矢量)
			// reset state : mx = 0
			// mx = mx + mK * mNu
			// mx = mK * mNu
			Matrix3x3d::mult(this->mK, this->mNu, this->mx);

			// M3 = mK * mH
			Matrix3x3d::mult(this->mK, this->mH, this->processAccTempM3);
			// M4 = I
			this->processAccTempM4.setIdentity();
			// M4 = I - mK * mH
			this->processAccTempM4.minusEquals(this->processAccTempM3);
			// M3 = (I - mK * mH) * mP
			Matrix3x3d::mult(this->processAccTempM4, this->mP,
					this->processAccTempM3);
			// 校正 P
			// mP = (I - mK * mH) * mP
			this->mP.set(this->processAccTempM3);

			// 从状态矢量 mx (代表旋转误差的旋转矢量)，求出微分旋转矩阵(姿态误差旋转矩阵)。
			// sO3FromMu : Process nonlinear vector function, f().
			// so3LastMotion 同时也是 F matrix
			So3Util::sO3FromMu(this->mx, this->so3LastMotion);

			// 姿态误差旋转矩阵 x 原先的姿态旋转矩阵 = 当前的姿态旋转矩阵
			Matrix3x3d::mult(this->so3LastMotion, this->so3SensorFromWorld,
					this->so3SensorFromWorld);

			// 预测 P
			// 运动后更新协方差矩阵 P = F P F'
			updateCovariancesAfterMotion();
		}
		else {
			// first
			// so3SensorFromWorld = down * currentAccel
			so3Helper.sO3FromTwoVec(this->down, this->mz, this->so3SensorFromWorld);
			this->alignedToGravity = true;
		}
	}

	public: void processMag(const float mag[3], const long sensorTimeStamp) {
		if (this->alignedToGravity) {
			return;
		}
		// 测量磁场矢量 mz，指向磁极。一般不位于水平面上，而是在水平面偏下指向磁极，磁倾角。
		this->mz.set((double)mag[0], (double)mag[1], (double)mag[2]);
		this->mz.normalize();

		Vector3d downInSensorFrame; // 当前姿态旋转矩阵分离出的重力矢量。
		this->so3SensorFromWorld.getColumn(2, downInSensorFrame);

		// 矢量 processMagTempV1 位于水平面，垂直于磁场矢量与重力矢量的共面。
		Vector3d::cross(this->mz, downInSensorFrame, this->processMagTempV1);
		// 矢量 perpToDownAndMag 是水平面上的一个矢量，垂直于指向磁极的矢量。
		// 右手规则：在北半球指向东，在南半球指向西。
		Vector3d& perpToDownAndMag = this->processMagTempV1;
		perpToDownAndMag.normalize();

		// 矢量 magHorizontal 是水平面上指向磁极的矢量。
		Vector3d::cross(downInSensorFrame, perpToDownAndMag,
				this->processMagTempV2);
		Vector3d& magHorizontal = this->processMagTempV2;

		magHorizontal.normalize();
		this->mz.set(magHorizontal);
		if (this->alignedToNorth) {
			// mag: 为了 Jacobian 数值矩阵的观测函数。为求 H matrix。
			// 观测函数 h() 计算机体坐标系中的参考矢量的预测姿态，
			// 并与测量矢量 z 计算差异得到创新矢量 ν。
			magObservationFunctionForNumericalJacobian(this->so3SensorFromWorld,
					this->mNu);

			const double eps = 1.0E-7D;
			// 按照列(坐标轴)更新测量矩阵 H, Jacobian matrix
			for (int dof = 0; dof < 3; dof++) {
				Vector3d& delta = this->processMagTempV3;
				delta.setZero();
				delta.setComponent(dof, eps);

				// 从微分角度矢量 ε，求出微分旋转矩阵。
				// sO3FromMu : Process nonlinear vector function, f().
				So3Util::sO3FromMu(delta, this->processMagTempM1);
				// 给当前的姿态旋转矩阵的一个坐标轴一个扰动 ε。
				Matrix3x3d::mult(this->processMagTempM1, this->so3SensorFromWorld,
						this->processMagTempM2);

				magObservationFunctionForNumericalJacobian(
						this->processMagTempM2, this->processMagTempV4);

				Vector3d& withDelta = this->processMagTempV4;

				Vector3d::sub(this->mNu, withDelta, this->processMagTempV5);
				this->processMagTempV5.scale(1.0D / eps);

				this->mH.setColumn(dof, this->processMagTempV5);
			}
			// M3 = mH'
			this->mH.transpose(this->processMagTempM3);
			// M4 = mP * mH'
			Matrix3x3d::mult(this->mP, this->processMagTempM3,
					this->processMagTempM4);
			// M5 = mH * mP * mH'
			Matrix3x3d::mult(this->mH, this->processMagTempM4,
					this->processMagTempM5);
			// mS = mH * mP * mH' + mR
			Matrix3x3d::add(this->processMagTempM5, this->mR, this->mS);

			// M3 = mS^-1
			this->mS.invert(this->processMagTempM3);
			// M4 = mH'
			this->mH.transpose(this->processMagTempM4);
			// M5 = mH' * mS^-1
			Matrix3x3d::mult(this->processMagTempM4, this->processMagTempM3,
					this->processMagTempM5);
			// mK = mP * mH' * mS^-1
			Matrix3x3d::mult(this->mP, this->processMagTempM5, this->mK);

			// 状态矢量 mx (代表旋转误差的旋转矢量)
			// reset state : mx = 0
			// mx = mx + mK * mNu
			// mx = mK * mNu
			Matrix3x3d::mult(this->mK, this->mNu, this->mx);

			// M3 = mK * mH
			Matrix3x3d::mult(this->mK, this->mH, this->processMagTempM3);
			// M4 = I
			this->processMagTempM4.setIdentity();
			// M4 = I - mK * mH
			this->processMagTempM4.minusEquals(this->processMagTempM3);
			// M3 = (I - mK * mH) * mP
			Matrix3x3d::mult(this->processMagTempM4, this->mP,
					this->processMagTempM3);
			// 校正 P
			// mP = (I - mK * mH) * mP
			this->mP.set(this->processMagTempM3);

			// 从状态矢量 mx (代表旋转误差的旋转矢量)，求出微分旋转矩阵(姿态误差旋转矩阵)。
			// sO3FromMu : Process nonlinear vector function, f().
			// so3LastMotion 同时也是 F matrix
			So3Util::sO3FromMu(this->mx, this->so3LastMotion);

			// 姿态误差旋转矩阵 x 原先的姿态旋转矩阵 = 当前的姿态旋转矩阵
			Matrix3x3d::mult(this->so3LastMotion, this->so3SensorFromWorld,
					this->processMagTempM3);
			this->so3SensorFromWorld.set(this->processMagTempM3);

			// 预测 P
			// 运动后更新协方差矩阵 P = F P F'
			updateCovariancesAfterMotion();
		}
		else {
			// first
			// mag: 为了 Jacobian 数值矩阵的观测函数
			magObservationFunctionForNumericalJacobian(this->so3SensorFromWorld,
					this->mNu);
			// 从状态矢量 mx (代表旋转误差的旋转矢量)，求出微分旋转矩阵(姿态误差旋转矩阵)。
			// sO3FromMu : Process nonlinear vector function, f().
			// so3LastMotion 同时也是 F matrix
			So3Util::sO3FromMu(this->mx, this->so3LastMotion);

			// 姿态误差微分旋转矩阵 x 原先的姿态旋转矩阵 = 当前的姿态旋转矩阵
			Matrix3x3d::mult(this->so3LastMotion, this->so3SensorFromWorld,
					this->processMagTempM3);
			this->so3SensorFromWorld.set(this->processMagTempM3);

			// 预测 P
			// 运动后更新协方差矩阵 P = F P F'
			updateCovariancesAfterMotion();
			this->alignedToNorth = true;
		}
	}

	private: double* glMatrixFromSo3(Matrix3x3d& so3) {
		/* 3 x 3 */
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				this->rotationMatrix[(4 * c + r)] = so3.get(r, c);
			}
		}
		// 4x4 齐次二次旋转矩阵。Fortran / OpenGL 格式
		//                       c   r
		this->rotationMatrix[4 * 0 + 3] = 0.0D;
		this->rotationMatrix[4 * 1 + 3] = 0.0D;
		this->rotationMatrix[4 * 2 + 3] = 0.0D;
		this->rotationMatrix[4 * 3 + 0] = 0.0D;
		this->rotationMatrix[4 * 3 + 1] = 0.0D;
		this->rotationMatrix[4 * 3 + 2] = 0.0D;
		this->rotationMatrix[4 * 3 + 3] = 1.0D;

		return this->rotationMatrix;
	}

	private: void filterGyroTimestep(float timeStep) {
		const float kFilterCoeff = 0.95F;
		const float kMinSamples = 10.00F;
		if (!this->timestepFilterInit) {
			this->filteredGyroTimestep = timeStep;
			this->numGyroTimestepSamples = 1;
			this->timestepFilterInit = true;
		}
		else {
			this->filteredGyroTimestep = (kFilterCoeff * this->filteredGyroTimestep + (1.0F - kFilterCoeff) * timeStep);
			if (++this->numGyroTimestepSamples > kMinSamples) {
				this->gyroFilterValid = true;
			}
		}
	}

	private: void updateCovariancesAfterMotion() {
		// so3LastMotion : 微分旋转矩阵。
		// so3LastMotion 同时也是 F matrix
		// M1 = so3LastMotion'
		this->so3LastMotion.transpose(this->updateCovariancesAfterMotionTempM1);
		// M2 = mP * so3LastMotion'
		Matrix3x3d::mult(this->mP, this->updateCovariancesAfterMotionTempM1,
				this->updateCovariancesAfterMotionTempM2);

		// mP = so3LastMotion * mP * so3LastMotion'
		// P = F P F'
		Matrix3x3d::mult(this->so3LastMotion,
				this->updateCovariancesAfterMotionTempM2, this->mP);
		// reset
		// so3LastMotion = I
		this->so3LastMotion.setIdentity();
	}

	private: void accObservationFunctionForNumericalJacobian(
			Matrix3x3d& so3SensorFromWorldPred, Vector3d& result) {
		// predict: 预测
		// so3SensorFromWorldPred: 预测姿态旋转矩阵
		// mh : 预测的重力矢量, h() is observation nonlinear vector function
		// down : 重力矢量，参考矢量。
		// mh = so3SensorFromWorldPred * down
		Matrix3x3d::mult(so3SensorFromWorldPred, this->down, this->mh);
		// 预测的 mh 与测量的 mz 有差异，求从 mh 到 mz 的差异旋转矩阵。
		// EKF 传统为 : mz - mh。旋转表示为 : mz = M * mh
		so3Helper.sO3FromTwoVec(this->mh, this->mz,
				this->accObservationFunctionForNumericalJacobianTempM);

		// resule = 从差异旋转矩阵求出代表创新的旋转矢量 ν
		so3Helper.muFromSO3(this->accObservationFunctionForNumericalJacobianTempM,
				result);
	}

	private: void magObservationFunctionForNumericalJacobian(
			Matrix3x3d& so3SensorFromWorldPred, Vector3d& result) {
		// predict: 预测
		// so3SensorFromWorldPred: 预测姿态旋转矩阵
		// north : 磁场矢量，参考矢量。
		// mh : 预测的磁场矢量, h() is observation nonlinear vector function
		// mh = so3SensorFromWorldPred * north
		Matrix3x3d::mult(so3SensorFromWorldPred, this->north, this->mh);
		// 预测的 mh 与测量的 mz 有差异，求从 mh 到 mz 的差异旋转矩阵。
		// EKF 传统为 : mz - mh。旋转表示为 : mz = M * mh
		so3Helper.sO3FromTwoVec(this->mh, this->mz,
				this->magObservationFunctionForNumericalJacobianTempM);

		// resule = 从差异旋转矩阵求出代表创新的旋转矢量 ν
		so3Helper.muFromSO3(this->magObservationFunctionForNumericalJacobianTempM,
				result);
	}
};

#endif // ORIENTATIONEKF_H
