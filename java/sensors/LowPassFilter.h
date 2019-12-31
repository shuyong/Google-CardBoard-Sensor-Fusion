#include <math.h>

#include "Vector3d.h"

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter {
    private: static constexpr double NANOS_TO_SECONDS = 1.0E-9F;
    private: double timeConstantSecs;
    private: Vector3d filteredData;
    private: long lastTimestampNs;
    private: int numSamples;
    private: Vector3d temp;

    public: LowPassFilter(double cutoffFrequency) {
        this->timeConstantSecs = 1.0D / (2 * M_PI * cutoffFrequency);
    }

    public: int getNumSamples() {
        return this->numSamples;
    }

    public: void addSample(Vector3d& sampleData, long timestampNs) {
        this->addWeightedSample(sampleData, timestampNs, 1.0D);
    }

    public: void addWeightedSample(Vector3d& sampleData, long timestampNs, double weight) {
        ++this->numSamples;
        if(this->numSamples == 1) {
            this->filteredData.set(sampleData);
            this->lastTimestampNs = timestampNs;
        } else {
            double weightedDeltaSecs = weight * (double)(timestampNs - this->lastTimestampNs) * NANOS_TO_SECONDS;
            double alpha = weightedDeltaSecs / (this->timeConstantSecs + weightedDeltaSecs);
            this->filteredData.scale(1.0D - alpha);
            this->temp.set(sampleData);
            this->temp.scale(alpha);
	    Vector3d::add(this->temp, this->filteredData, this->filteredData);
            this->lastTimestampNs = timestampNs;
        }
    }

    public: Vector3d& getFilteredData() {
        return this->filteredData;
    }

};

#endif // LOWPASSFILTER_H
