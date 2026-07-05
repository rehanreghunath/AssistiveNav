#include "ImuFusion.h"
#include <android/log.h>
#include <cmath>
#include <algorithm>

#define LOG_TAG "ImuFusion"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    ImuFusion::ImuFusion(int width, int height)
            : mWidth(width),
              mHeight(height),
              mFocalLengthPx(kFallbackFocalLengthPx),
              mLastAngVelRadPerSec(0.0f),
              mReadIdx(0),
              mHasData(false),
              mPrevTimestampNs(0),
              mHasPrev(false),
              mHasInitialQuat(false)
    {
        mSlot[0] = {0.0f, 0.0f, 0.0f, 1.0f};
        mSlot[1] = {0.0f, 0.0f, 0.0f, 1.0f};
        mPrevQuat    = {0.0f, 0.0f, 0.0f, 1.0f};
        mInitialQuat = {0.0f, 0.0f, 0.0f, 1.0f};
        LOGI("Created for %dx%d, focal length %.0f px (fallback)",
             mWidth, mHeight, mFocalLengthPx);
    }

    void ImuFusion::setFocalLength(float focalLengthPx) {
        if (focalLengthPx < kMinPlausibleFx || focalLengthPx > kMaxPlausibleFx) {
            LOGW("setFocalLength: %.1f px out of range — keeping fallback", focalLengthPx);
            return;
        }
        mFocalLengthPx = focalLengthPx;
        LOGI("Focal length set to %.1f px", mFocalLengthPx);
    }

    // ── Quaternion helpers ────────────────────────────────────────────────────

    ImuFusion::Quaternion ImuFusion::multiplyQuat(const Quaternion& a, const Quaternion& b) {
        return {
                a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
                a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
                a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
        };
    }

    ImuFusion::Quaternion ImuFusion::conjugateQuat(const Quaternion& q) {
        return {-q.x, -q.y, -q.z, q.w};
    }

    void ImuFusion::quatToRotMat(const Quaternion& q, float out[9]) {
        const float x = q.x, y = q.y, z = q.z, w = q.w;
        const float xx = x*x, yy = y*y, zz = z*z;
        const float xy = x*y, xz = x*z, yz = y*z;
        const float wx = w*x, wy = w*y, wz = w*z;

        out[0] = 1.f-2.f*(yy+zz); out[1] = 2.f*(xy-wz);     out[2] = 2.f*(xz+wy);
        out[3] = 2.f*(xy+wz);     out[4] = 1.f-2.f*(xx+zz); out[5] = 2.f*(yz-wx);
        out[6] = 2.f*(xz-wy);     out[7] = 2.f*(yz+wx);     out[8] = 1.f-2.f*(xx+yy);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  getRotationMatrix
    //
    //  Returns R_relative = q_current * conj(q_initial) as a rotation matrix.
    //  This gives the camera orientation relative to session start, which is
    //  the correct world frame for PTAM (identity at time 0).
    // ─────────────────────────────────────────────────────────────────────────

    bool ImuFusion::getRotationMatrix(float outR[9]) const {
        if (!mHasData.load(std::memory_order_acquire) || !mHasInitialQuat)
            return false;

        const Quaternion curr = mSlot[mReadIdx.load(std::memory_order_acquire)];
        // Relative rotation: how much has the device rotated since session start?
        const Quaternion rel  = multiplyQuat(curr, conjugateQuat(mInitialQuat));
        quatToRotMat(rel, outR);
        return true;
    }

    void ImuFusion::predictedDisplacement(const float H[9],
                                          float px, float py,
                                          float cx, float cy,
                                          float& outDx, float& outDy) const {
        const float hx = H[0]*px + H[1]*py + H[2];
        const float hy = H[3]*px + H[4]*py + H[5];
        const float hw = H[6]*px + H[7]*py + H[8];

        if (std::abs(hw) < 1e-6f) { outDx = outDy = 0.0f; return; }
        const float invW = 1.0f / hw;
        outDx = hx * invW - px;
        outDy = hy * invW - py;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  updateRotation  — sensor thread (unchanged logic, adds initial quat)
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::updateRotation(float qx, float qy, float qz, float qw,
                                   int64_t /* timestampNs */) {
        const float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (norm < 1e-6f) return;

        const float invN    = 1.0f / norm;
        const int writeSlot = 1 - mReadIdx.load(std::memory_order_relaxed);
        mSlot[writeSlot]    = {qx*invN, qy*invN, qz*invN, qw*invN};
        mReadIdx.store(writeSlot, std::memory_order_release);
        mHasData.store(true,      std::memory_order_release);

        // Record the very first valid quaternion as the session-start reference.
        // All subsequent getRotationMatrix() calls return the delta from this.
        if (!mHasInitialQuat) {
            mInitialQuat    = mSlot[writeSlot];
            mHasInitialQuat = true;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  compensate  — camera executor thread (unchanged from prior version)
    // ─────────────────────────────────────────────────────────────────────────

    void ImuFusion::compensate(FlowResult& result) {
        if (!mHasData.load(std::memory_order_acquire)) return;

        const Quaternion currQuat =
                mSlot[mReadIdx.load(std::memory_order_acquire)];

        if (!mHasPrev) {
            mPrevQuat            = currQuat;
            mHasPrev             = true;
            mLastAngVelRadPerSec = 0.0f;
            return;
        }

        const Quaternion qDelta = multiplyQuat(currQuat, conjugateQuat(mPrevQuat));

        {
            const float halfAngle    = std::acos(
                    std::clamp(std::abs(qDelta.w), 0.0f, 1.0f));
            mLastAngVelRadPerSec = halfAngle * 2.0f * kAssumedFrameHz;
        }

        if (qDelta.w > 0.9999990f) { mPrevQuat = currQuat; return; }

        float R[9];
        quatToRotMat(qDelta, R);

        const float f    = mFocalLengthPx;
        const float cx   = static_cast<float>(mWidth)  * 0.5f;
        const float cy   = static_cast<float>(mHeight) * 0.5f;
        const float invF = 1.0f / f;

        float RKinv[9];
        for (int row = 0; row < 3; ++row) {
            RKinv[row*3+0] =  R[row*3+0] * invF;
            RKinv[row*3+1] =  R[row*3+1] * invF;
            RKinv[row*3+2] = -R[row*3+0]*cx*invF - R[row*3+1]*cy*invF + R[row*3+2];
        }

        float H[9];
        for (int col = 0; col < 3; ++col) {
            H[0*3+col] = f   * RKinv[0*3+col] + cx * RKinv[2*3+col];
            H[1*3+col] = f   * RKinv[1*3+col] + cy * RKinv[2*3+col];
            H[2*3+col] =                              RKinv[2*3+col];
        }

        float magSum = 0.0f;
        for (FlowVector& fv : result.vectors) {
            float predDx, predDy;
            predictedDisplacement(H, fv.x0, fv.y0, cx, cy, predDx, predDy);
            fv.dx       -= predDx;
            fv.dy       -= predDy;
            fv.magnitude = std::sqrt(fv.dx*fv.dx + fv.dy*fv.dy);
            fv.angle     = std::atan2(fv.dy, fv.dx);
            magSum      += fv.magnitude;
        }

        result.globalMeanMag = result.trackedCount > 0
                               ? magSum / static_cast<float>(result.trackedCount)
                               : 0.0f;

        mPrevQuat = currQuat;
    }

} // namespace assistivenav