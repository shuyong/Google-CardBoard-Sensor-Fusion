#include "Vector3d.h"
#include "LowPassFilter.h"

#ifndef GYROSCOPEBIASESTIMATOR_H
#define GYROSCOPEBIASESTIMATOR_H

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

class GyroscopeBiasEstimator {
    private: static constexpr float ACCEL_LOWPASS_FREQ = 1.0F;
    private: static constexpr float GYRO_LOWPASS_FREQ = 10.0F;
    private: static constexpr float GYRO_BIAS_LOWPASS_FREQ = 0.15F;
    private: static constexpr int NUM_GYRO_BIAS_SAMPLES_THRESHOLD = 30;
    private: static constexpr int NUM_GYRO_BIAS_SAMPLES_INITIAL_SMOOTHING = 100;
    private: LowPassFilter accelLowPass(ACCEL_LOWPASS_FREQ);
    private: LowPassFilter gyroLowPass(GYRO_LOWPASS_FREQ);
    private: LowPassFilter gyroBiasLowPass(GYRO_BIAS_LOWPASS_FREQ);
    private: static constexpr float ACCEL_DIFF_STATIC_THRESHOLD = 0.5F;
    private: static constexpr float GYRO_DIFF_STATIC_THRESHOLD = 0.008F;
    private: Vector3d smoothedGyroDiff;
    private: Vector3d smoothedAccelDiff;
    private: static constexpr float GYRO_FOR_BIAS_THRESHOLD = 0.35F;
    private: static constexpr int IS_STATIC_NUM_FRAMES_THRESHOLD = 10;
    private: GyroscopeBiasEstimator::IsStaticCounter isAccelStatic(IS_STATIC_NUM_FRAMES_THRESHOLD);
    private: GyroscopeBiasEstimator::IsStaticCounter isGyroStatic(IS_STATIC_NUM_FRAMES_THRESHOLD);

    public: GyroscopeBiasEstimator() {
        this->reset();
    }

    public: void reset() {
    }

    public: void processGyroscope(Vector3d& gyro, long sensorTimestampNs) {
        this->gyroLowPass.addSample(gyro, sensorTimestampNs);
	Vector3d::sub(gyro, this->gyroLowPass.getFilteredData(), this->smoothedGyroDiff);
        this->isGyroStatic.appendFrame(this->smoothedGyroDiff.length() < GYRO_DIFF_STATIC_THRESHOLD);
        if(this->isGyroStatic.isRecentlyStatic() && this->isAccelStatic.isRecentlyStatic()) {
            this->updateGyroBias(gyro, sensorTimestampNs);
        }

    }

    public: void processAccelerometer(Vector3d& accel, long sensorTimestampNs) {
        this->accelLowPass.addSample(accel, sensorTimestampNs);
	Vector3d::sub(accel, this->accelLowPass.getFilteredData(), this->smoothedAccelDiff);
        this->isAccelStatic.appendFrame(this->smoothedAccelDiff.length() < ACCEL_DIFF_STATIC_THRESHOLD);
    }

    public: void getGyroBias(Vector3d& result) {
        if(this->gyroBiasLowPass.getNumSamples() < NUM_GYRO_BIAS_SAMPLES_THRESHOLD) {
            result.setZero();
        } else {
            result.set(this->gyroBiasLowPass.getFilteredData());
            double rampUpRatio = min(1.0D, (double)(this->gyroBiasLowPass.getNumSamples() - NUM_GYRO_BIAS_SAMPLES_THRESHOLD) / NUM_GYRO_BIAS_SAMPLES_INITIAL_SMOOTHING);
            result.scale(rampUpRatio);
        }

    }

    private: void updateGyroBias(Vector3d& gyro, long sensorTimestampNs) {
        if(gyro.length() < GYRO_FOR_BIAS_THRESHOLD) {
            double updateWeight = max(0.0D, 1.0D - gyro.length() / GYRO_FOR_BIAS_THRESHOLD);
            updateWeight *= updateWeight;
            this->gyroBiasLowPass.addWeightedSample(this->gyroLowPass.getFilteredData(), sensorTimestampNs, updateWeight);
        }
    }

    private: class IsStaticCounter {
        private: int minStaticFrames;
        private: int consecutiveIsStatic;

        IsStaticCounter(int minStaticFrames) {
            this->minStaticFrames = minStaticFrames;
        }

        void appendFrame(boolean isStatic) {
            if(!isStatic) {
                this->consecutiveIsStatic = 0;
            } else {
                ++this->consecutiveIsStatic;
            }

        }

        boolean isRecentlyStatic() {
            return this->consecutiveIsStatic >= this->minStaticFrames;
        }
    }
}

#endif // GYROSCOPEBIASESTIMATOR_H
