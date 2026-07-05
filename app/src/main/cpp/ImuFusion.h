#pragma once

#include "FlowTypes.h"
#include <cstdint>
#include <atomic>

namespace assistivenav {

    class ImuFusion {
    public:
        ImuFusion(int width, int height);

        ~ImuFusion() = default;
        ImuFusion(const ImuFusion&)            = delete;
        ImuFusion& operator=(const ImuFusion&) = delete;

        void setFocalLength(float focalLengthPx);

        void updateRotation(float qx, float qy, float qz, float qw,
                            int64_t timestampNs);

        void compensate(FlowResult& result);

        float focalLengthPx()        const { return mFocalLengthPx; }
        float rotationRateRadPerSec() const { return mLastAngVelRadPerSec; }

        /** Fill outR (row-major float[9]) with the rotation matrix representing
         *  the device orientation RELATIVE TO THE FIRST RECEIVED READING.
         *  This is the world-to-camera rotation in PTAM's world frame.
         *  Returns false when no sensor data has been received yet. */
        bool getRotationMatrix(float outR[9]) const;

    private:
        const int mWidth;
        const int mHeight;

        static constexpr float kFallbackFocalLengthPx = 500.0f;
        static constexpr float kMinPlausibleFx        = 100.0f;
        static constexpr float kMaxPlausibleFx        = 5000.0f;
        static constexpr float kAssumedFrameHz        = 30.0f;

        float mFocalLengthPx;
        float mLastAngVelRadPerSec;

        struct Quaternion { float x, y, z, w; };

        Quaternion           mSlot[2];
        std::atomic<int>     mReadIdx;
        std::atomic<bool>    mHasData;

        int64_t              mPrevTimestampNs;
        Quaternion           mPrevQuat;
        bool                 mHasPrev;

        // Records the very first quaternion so getRotationMatrix() returns a
        // pose relative to session start rather than relative to magnetic north.
        Quaternion           mInitialQuat;
        bool                 mHasInitialQuat;

        static Quaternion multiplyQuat(const Quaternion& a, const Quaternion& b);
        static Quaternion conjugateQuat(const Quaternion& q);
        static void       quatToRotMat(const Quaternion& q, float out[9]);

        void predictedDisplacement(const float H[9],
                                   float px, float py,
                                   float cx, float cy,
                                   float& outDx, float& outDy) const;
    };

} // namespace assistivenav