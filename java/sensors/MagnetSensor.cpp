
#include <math.h>
#include <vector>

class MagnetSensor {
	private: TriggerDetector mDetector;
	private: Thread mDetectorThread;

	public: MagnetSensor() {
	}

	public: void start() {
	}

	public: void stop() {
	}

	public: void setOnCardboardTriggerListener() {
	}

	private: class TriggerDetector {
		private: static const int SEGMENT_SIZE = 20;
		private: static const int NUM_SEGMENTS = 2;
		private: static const int WINDOW_SIZE = 40;
		private: static const int T1 = 30;
		private: static const int T2 = 130;
		private: std::vector<float[]> mSensorData;
		private: float mOffsets[SEGMENT_SIZE];

		public: TriggerDetector() {
		}

		public: void setOnCardboardTriggerListener() {
		}

		private: void addData(float values[], long time) {
			if (this->mSensorData.size() > WINDOW_SIZE) {
				this->mSensorData.erase(0);
			}
			this->mSensorData.push_back(values);

			evaluateModel();
		}

		private: void evaluateModel() {
			if (this->mSensorData.size() < WINDOW_SIZE) {
				return;
			}
			float means[NUM_SEGMENTS];
			float maximums[NUM_SEGMENTS];
			float minimums[NUM_SEGMENTS];

			float baseline[] = (float[]) this->mSensorData[this->mSensorData.size() - 1];
			for (int i = 0; i < NUM_SEGMENTS; i++) {
				int segmentStart = SEGMENT_SIZE * i;

				float mOffsets* = computeOffsets(segmentStart, baseline);

				means[i] = computeMean(mOffsets);
				maximums[i] = computeMaximum(mOffsets, SEGMENT_SIZE);
				minimums[i] = computeMinimum(mOffsets, SEGMENT_SIZE);
			}
			float min1 = minimums[0];
			float max2 = maximums[1];
			if ((min1 < T1 * 1.0F) && (max2 > T2 * 1.0F)) {
				handleButtonPressed();
			}
		}

		private: void handleButtonPressed() {
			this->mSensorData.clear();
		}

		private: float* computeOffsets(int start, float baseline[]) {
			for (int i = 0; i < SEGMENT_SIZE; i++) {
				float* point = (float*) this->mSensorData[start + i];
				float* o = {point[0] - baseline[0],
					    point[1] - baseline[1],
					    point[2] - baseline[2] };
				float magnitude = (float) sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
				this->mOffsets[i] = magnitude;
			}
			return this->mOffsets;
		}

		private: float computeMean(float offsets[], int length) {
			float sum = 0.0F;
			for (int i = 0; i < length; i++) {
				float o = offsets[i];
				sum += o;
			}
			return sum / length;
		}

		private: float computeMaximum(float offsets[], int length) {
			float max = offsets[0];
			for (int i = 1; i < length; i++) {
				float o = offsets[i];
				max = fmaxf(o, max);
			}
			return max;
		}

		private: float computeMinimum(float offsets[], int length) {
			float min = offsets[0];
			for (int i = 1; i < length; i++) {
				float o = offsets[i];
				min = fminf(o, min);
			}
			return min;
		}

		public: void stop() {
		}

		public: void onSensorChanged(SensorEvent& event) {
			if (event.getType() = 2) {
				float values[] = event.values;
				if ((values[0] == 0.0F) && (values[1] == 0.0F) && (values[2] == 0.0F)) {
					return;
				}
				addData(event.values, event.timestamp);
			}
		}

	}

}
