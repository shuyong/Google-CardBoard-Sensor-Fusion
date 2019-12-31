#include <stdio.h>

#include "Matrix.h"
#include "OrientationEKF.h"
#include "SensorEvent.h"

extern "C" uint64_t nanoTime();

#ifndef HEADTRACKER_H
#define HEADTRACKER_H

// 头部姿态跟踪系统
// 用 Error State Kalman Filter 实现
// 核心运算元素为 4x4 rotation matrix
// 机体坐标系(body frame): Android(ENU)
class HeadTracker {
	private: static constexpr double NS2S = 1.0E-9D;
	private: static constexpr float DEFAULT_NECK_HORIZONTAL_OFFSET = 0.08F;
	private: static constexpr float DEFAULT_NECK_VERTICAL_OFFSET = 0.075F;
	private: static constexpr bool  DEFAULT_NECK_MODEL_ENABLED = false;
	private: float mEkfToHeadTracker[16];
	private: float mSensorToDisplay[16];
	private: float mDisplayRotation = NAN;
	private: float mNeckModelTranslation[16];
	private: float mTmpHeadView[16];
	private: float mTmpHeadView2[16];
	private: bool mNeckModelEnabled = DEFAULT_NECK_MODEL_ENABLED;	// 是否使能低头颈部模型
	private: volatile bool mTracking;
	private: OrientationEKF mTracker;
	private: long mLatestGyroEventClockTimeNs;
	private: Vector3d mGyroBias;
	private: Vector3d mLatestGyro;
	private: Vector3d mLatestAcc;
	private: float mLatestMag[3];

	public: HeadTracker() {
		Matrix::setIdentityM(this->mNeckModelTranslation, 0);
		Matrix::translateM(this->mNeckModelTranslation, 0,
                0.0F, -DEFAULT_NECK_VERTICAL_OFFSET, DEFAULT_NECK_HORIZONTAL_OFFSET);
	}

	// 陀螺仪、加速计和磁力计传感器数据输入接口
	public: void onSensorChanged(const SensorEvent& event) {
		if (this->mTracking) {
			if (event.type == SENSOR_TYPE_ACCELEROMETER) {
				this->mLatestAcc.set(event.values[0], event.values[1], event.values[2]);
				this->mTracker.processAcc(this->mLatestAcc, event.timestamp);
			}
			else if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
				this->mLatestMag[0] = event.values[0];
				this->mLatestMag[1] = event.values[1];
				this->mLatestMag[2] = event.values[2];
				this->mTracker.processMag(this->mLatestMag, event.timestamp);
			}
			else if (event.type == SENSOR_TYPE_GYROSCOPE) {
				this->mLatestGyroEventClockTimeNs = nanoTime();
				this->mLatestGyro.set(event.values[0], event.values[1], event.values[2]);
				Vector3d::sub(this->mLatestGyro, this->mGyroBias, this->mLatestGyro);
				this->mTracker.processGyro(this->mLatestGyro, event.timestamp);
			}
		}
	}

	// 开始头部跟踪
	public: void startTracking() {
		if (this->mTracking) {
			return;
		}
		this->mTracker.reset();
		this->mTracking = true;
	}

	// 结束头部跟踪
	public: void stopTracking() {
		if (!this->mTracking) {
			return;
		}
		this->mTracking = false;
	}

	// 设置陀螺仪偏差
	public: void setGyroBias(const float gyroBias[3]) {
		this->mGyroBias.set(gyroBias[0], gyroBias[1], gyroBias[2]);
	}

	// 使能低头颈部模型
	public: void setNeckModelEnabled(const bool enabled) {
        	this->mNeckModelEnabled = enabled;
	}

	// 获取最新 4x4 rotation matrix
	public: void getLastHeadView(float headView[], int length, int offset) {
		if (offset + 16 > length) {
			fprintf(stderr, "Not enough space to write the result\n");
			return;
		}
		float rotation = 0.0F;
#if 0
		rotation = 0.0F;
		rotation = 90.0F;
		rotation = 180.0F;
		rotation = 270.0F;
#endif
		if (rotation != this->mDisplayRotation) {
			this->mDisplayRotation = rotation;
			Matrix::setRotateEulerM(this->mSensorToDisplay , 0,   0.0F, 0.0F, -rotation);
			Matrix::setRotateEulerM(this->mEkfToHeadTracker, 0, -90.0F, 0.0F,  rotation);
		}
		if (this->mTracker.isReady()) {
			const double secondsSinceLastGyroEvent = (nanoTime() - this->mLatestGyroEventClockTimeNs) * NS2S;
			const double secondsToPredictForward = secondsSinceLastGyroEvent + 1.0D / 30.0D; // 30HZ
			const double *mat = this->mTracker.getPredictedGLMatrix(secondsToPredictForward);// 预测下一个屏幕刷新VSYNC时刻的姿态
			for (int i = 0; i < length; i++) {
				this->mTmpHeadView[i] = ((float) mat[i]);
			}
		}
		Matrix::multiplyMM(this->mTmpHeadView2, 0, this->mSensorToDisplay, 0, this->mTmpHeadView, 0);
		Matrix::multiplyMM(headView, offset, this->mTmpHeadView, 0, this->mEkfToHeadTracker, 0);
		// 使用低头颈部模型
		if (this->mNeckModelEnabled) {
			Matrix::multiplyMM(this->mTmpHeadView, 0, this->mNeckModelTranslation, 0, headView, offset);
			Matrix::translateM(headView, offset, this->mTmpHeadView, 0, 0.0F, DEFAULT_NECK_VERTICAL_OFFSET, 0.0F);
        	}
	}
};

#endif // HEADTRACKER_H
