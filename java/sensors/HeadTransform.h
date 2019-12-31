#include <stdio.h>
#include <string.h>
#include <math.h>

#include "Matrix.h"

#ifndef HEADTRANSFORM_H
#define HEADTRANSFORM_H

// convert from 4x4 rotation matrix to
class HeadTransform {
    private: static constexpr float GIMBAL_LOCK_EPSILON = 0.01F;
    private: float mHeadView[16];
    
    public: HeadTransform() {
	Matrix::setIdentityM(this->mHeadView, 0);
    }
    
    float* getHeadView() {
        return this->mHeadView;
    }
    
    // 4x4 rotation matrix
    public: void getHeadView(float headView[], int length, int offset) {
        if (offset + 16 > length) {
            fprintf(stderr, "Not enough space to write the result\n");
	    return;
        }
	memcpy(&headView[offset], this->mHeadView, 16 * sizeof(float));
    }
    
    // forward vector
    //  00   01   02   03
    // [  ] [  ] [  ] [  ] 00
    // [  ] [  ] [  ] [  ] 01
    // [08] [09] [10] [  ] 02
    // [  ] [  ] [  ] [  ] 03
    public: void getForwardVector(float forward[], int length, int offset) {
        if (offset + 3 > length) {
            fprintf(stderr, "Not enough space to write the result\n");
	    return;
        }
        for (int i = 0; i < 3; ++i) {
            forward[i + offset] = -this->mHeadView[8 + i];
        }
    }
    
    // up vector
    //  00   01   02   03
    // [  ] [  ] [  ] [  ] 00
    // [04] [05] [06] [  ] 01
    // [  ] [  ] [  ] [  ] 02
    // [  ] [  ] [  ] [  ] 03
    public: void getUpVector(float up[], int length, int offset) {
        if (offset + 3 > length) {
            fprintf(stderr, "Not enough space to write the result\n");
	    return;
        }
        for (int i = 0; i < 3; ++i) {
            up[i + offset] = this->mHeadView[4 + i];
        }
    }
    
    // right vector
    //  00   01   02   03
    // [00] [01] [02] [  ] 00
    // [  ] [  ] [  ] [  ] 01
    // [  ] [  ] [  ] [  ] 02
    // [  ] [  ] [  ] [  ] 03
    public: void getRightVector(float right[], int length, int offset) {
        if (offset + 3 > length) {
            fprintf(stderr, "Not enough space to write the result\n");
	    return;
        }
        for (int i = 0; i < 3; ++i) {
            right[i + offset] = this->mHeadView[i];
        }
    }
    
    // rotation matrix to quaternion
    public: void getQuaternion(float quaternion[], int length, int offset) {
        if (offset + 4 > length) {
            fprintf(stderr, "Not enough space to write the result\n");
	    return;
        }
        const float* m = this->mHeadView;
        const float t = m[0] + m[5] + m[10];
        float w;
        float x;
        float y;
        float z;
        if (t >= 0.0F) {
            float s = sqrtf(t + 1.0F);
            w = 0.5F * s;
            s = 0.5F / s;
            x = (m[9] - m[6]) * s;
            y = (m[2] - m[8]) * s;
            z = (m[4] - m[1]) * s;
        }
        else if (m[0] > m[5] && m[0] > m[10]) {
            float s = sqrtf(1.0F + m[0] - m[5] - m[10]);
            x = s * 0.5F;
            s = 0.5F / s;
            y = (m[4] + m[1]) * s;
            z = (m[2] + m[8]) * s;
            w = (m[9] - m[6]) * s;
        }
        else if (m[5] > m[10]) {
            float s = sqrtf(1.0F + m[5] - m[0] - m[10]);
            y = s * 0.5F;
            s = 0.5F / s;
            x = (m[4] + m[1]) * s;
            z = (m[9] + m[6]) * s;
            w = (m[2] - m[8]) * s;
        }
        else {
            float s = sqrtf(1.0F + m[10] - m[0] - m[5]);
            z = s * 0.5F;
            s = 0.5F / s;
            x = (m[2] + m[8]) * s;
            y = (m[9] + m[6]) * s;
            w = (m[4] - m[1]) * s;
        }
	// JPL convention
        quaternion[offset + 0] = x;
        quaternion[offset + 1] = y;
        quaternion[offset + 2] = z;
        quaternion[offset + 3] = w;
    }
    
    public: void getEulerAngles(float eulerAngles[], int length, int offset) {
        if (offset + 3 > length) {
            fprintf(stderr, "Not enough space to write the result\n");
	    return;
        }
        const float pitch = (float)asin(this->mHeadView[6]);
        float yaw;
        float roll;
        if (sqrtf(1.0F - this->mHeadView[6] * this->mHeadView[6]) >= GIMBAL_LOCK_EPSILON) {
            yaw  = (float)atan2(-this->mHeadView[2], this->mHeadView[10]);
            roll = (float)atan2(-this->mHeadView[4], this->mHeadView[ 5]);
        }
        else {
            yaw  = 0.0F;
            roll = (float)atan2( this->mHeadView[1], this->mHeadView[ 0]);
        }
        eulerAngles[offset + 0] = -pitch;
        eulerAngles[offset + 1] = -yaw;
        eulerAngles[offset + 2] = -roll;
    }
};

#endif
