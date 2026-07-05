#pragma once

#include "PTAMTypes.h"
#include "FlowTypes.h"

#include <opencv2/core.hpp>
#include <array>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace assistivenav {

// ─────────────────────────────────────────────────────────────────────────────
// PTAMSystem
//
// A simplified PTAM (Parallel Tracking And Mapping) for monocular obstacle
// mapping on a mobile device.
//
// TRACKING THREAD (camera executor, per frame):
//   Uses solvePnPRansac to estimate the camera translation by matching
//   active 3D map points to the observed 2D feature positions in the current
//   frame.  IMU rotation is used as the rotation component of the pose and as
//   an initial guess, so PnP only needs to solve for translation.
//   Falls back to the last valid pose when the map is empty or PnP fails.
//
// MAPPING THREAD (background, triggered by keyframe submission):
//   Receives candidate keyframes from the camera thread.  When two keyframes
//   with sufficient baseline exist, it calls cv::triangulatePoints to produce
//   new 3D world-frame map points, filters them by reprojection error and
//   depth, then inserts them into the shared map.
//
// Scale:
//   Monocular triangulation cannot determine absolute metric scale.  All
//   poses and map points use the same relative coordinate system, where one
//   "PTAM unit" equals the distance the camera travelled between the first
//   two accepted keyframes.  AudioEngine works in normalised angles and
//   relative proximity, so absolute scale is not required.
//
// Thread-safety model:
//   mMap, mMapSize   — protected by mMapMutex (both threads access)
//   mKeyFrames,
//   mKfCount,
//   mCurrentPose,
//   mFrameCount      — camera thread only (no locking needed)
//   mMapRequest      — protected by mReqMutex (producer/consumer hand-off)
// ─────────────────────────────────────────────────────────────────────────────

    class PTAMSystem {
    public:
        PTAMSystem(int width, int height, float fx, float fy, float cx, float cy);
        ~PTAMSystem();

        PTAMSystem(const PTAMSystem&)            = delete;
        PTAMSystem& operator=(const PTAMSystem&) = delete;

        // ── Tracker API (camera executor thread) ──────────────────────────────────

        /** Estimate the current camera pose.
         *  flow   — rotation-compensated flow vectors from ImuFusion::compensate().
         *  imuR   — absolute IMU rotation matrix (world→camera, row-major float[9])
         *           obtained from ImuFusion::getRotationMatrix().
         *  Returns the estimated Pose3D; the rotation component is taken from imuR
         *  and the translation is solved by PnP against the 3D map. */
        Pose3D processFrame(const FlowResult& flow, const float imuR[9]);

        // ── Map read access (hold mapMutex while reading) ─────────────────────────

        std::mutex&       mapMutex()  { return mMapMutex; }
        const MapPoint3D* mapPoints() const { return mMap.data(); }
        int               mapSize()   const { return mMapSize; }

    private:
        // ── Intrinsics ────────────────────────────────────────────────────────────
        const int   mWidth, mHeight;
        const float mFx, mFy, mCx, mCy;

        // ── Tracking state (camera thread only) ───────────────────────────────────
        Pose3D mCurrentPose;
        int    mFrameCount;

        // Pre-allocated PnP buffers — avoids vector construction each frame.
        // solvePnPRansac allocates internally, but keeping mPnpObj/mPnpImg
        // pre-reserved prevents repeated growth.
        std::vector<cv::Point3f> mPnpObj;
        std::vector<cv::Point2f> mPnpImg;
        cv::Mat                  mRvec, mTvec, mInliers;

        // ── Keyframe ring buffer (camera thread only) ──────────────────────────────
        std::array<KeyFrame3D, kMaxKeyFrames> mKeyFrames;
        int mKfCount;   // monotonically increasing; ring index = mKfCount % kMaxKeyFrames

        // ── Shared 3D map ──────────────────────────────────────────────────────────
        std::array<MapPoint3D, kMaxMapPoints> mMap;
        int        mMapSize;
        std::mutex mMapMutex;

        // ── Mapper thread hand-off ─────────────────────────────────────────────────
        // Copies of the current and previous keyframe are sent together so the
        // mapper thread never reads mKeyFrames (which the camera thread owns).
        struct MapRequest {
            KeyFrame3D curr;
            KeyFrame3D prev;
            bool       hasPrev = false;
            bool       ready   = false;
        } mMapRequest;
        std::mutex              mReqMutex;
        std::condition_variable mReqCv;
        std::thread             mMapperThread;
        std::atomic<bool>       mRunning;

        // ── Tuning constants ───────────────────────────────────────────────────────

        // PnP: how many 3D-2D correspondences are required before attempting PnP.
        static constexpr int   kMinPnPPts        = 6;
        // Reprojection error threshold for PnP RANSAC inlier classification.
        static constexpr float kPnPReprojPx      = 4.0f;
        // Search radius when matching a projected map point to a nearby flow origin.
        static constexpr float kMatchRadiusPx    = 25.0f;

        // Keyframe selection: minimum frames between keyframes prevents redundant
        // views and keeps the mapper thread from being overwhelmed.
        static constexpr int   kMinKfGapFrames   = 15;
        // Minimum number of feature vectors for a frame to qualify as a keyframe.
        static constexpr int   kMinFeaturesForKF = 30;
        // Minimum translation (in PTAM units) between consecutive keyframes.
        // Prevents adding keyframes when the user is stationary.
        static constexpr float kMinKfBaseline    = 0.03f;

        // Triangulation: accepted depth range in PTAM units.
        static constexpr float kMinDepth         = 0.05f;
        static constexpr float kMaxDepth         = 50.0f;
        // Accepted reprojection error for a freshly triangulated point.
        static constexpr float kMaxReprojPx      = 3.0f;
        // Feature matching radius used when correlating points across two keyframes.
        static constexpr float kMatchRadForTri   = 20.0f;
        // Minimum distance between a new point and any existing map point.
        // Suppresses duplicates that arise from repeated triangulation of the same
        // scene feature via different keyframe pairs.
        static constexpr float kDuplicateRadius2 = 0.01f;

        // Map management: confidence decays each frame so stale points fade out
        // when the user moves away from an area.
        static constexpr float kDecayFactor      = 0.99f;
        static constexpr float kMinConfForKeep   = 0.05f;

        // ── Tracker helpers ────────────────────────────────────────────────────────
        bool  projectPoint(const Pose3D& pose, const float pt3[3],
                           float& u, float& v) const;
        bool  findNearestFlow(const FlowResult& flow, float u, float v,
                              int& outIdx) const;
        Pose3D runPnP(const FlowResult& flow);

        // ── Keyframe management (camera thread) ────────────────────────────────────
        bool shouldAddKeyFrame(const FlowResult& flow) const;
        void submitKeyFrame(const FlowResult& flow);

        // ── Mapper helpers (mapper thread) ─────────────────────────────────────────
        void mapperLoop();
        void triangulateAndAdd(const KeyFrame3D& kfA, const KeyFrame3D& kfB);
        bool reprojError(const float pt3[3], const Pose3D& pose,
                         float u2d, float v2d, float& errOut) const;
        int  insertMapPoint(const float pos[3]);
    };

} // namespace assistivenav