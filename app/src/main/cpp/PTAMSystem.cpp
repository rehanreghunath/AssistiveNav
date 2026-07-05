#include "PTAMSystem.h"

#include <android/log.h>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>
#include <cstring>

#define LOG_TAG "PTAMSystem"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)

namespace assistivenav {

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

    PTAMSystem::PTAMSystem(int width, int height,
                           float fx, float fy, float cx, float cy)
            : mWidth(width), mHeight(height),
              mFx(fx), mFy(fy), mCx(cx), mCy(cy),
              mFrameCount(0), mMapSize(0), mKfCount(0),
              mRunning(true)
    {
        mCurrentPose = Pose3D{};
        mMap.fill(MapPoint3D{});

        // Pre-reserve buffers so that the first PnP call does not trigger
        // vector reallocation mid-frame.
        mPnpObj.reserve(kMaxMapPoints);
        mPnpImg.reserve(kMaxMapPoints);

        mMapRequest.ready = false;
        mMapperThread = std::thread(&PTAMSystem::mapperLoop, this);

        LOGI("Created %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
             width, height, fx, fy, cx, cy);
    }

    PTAMSystem::~PTAMSystem() {
        // Signal mapper thread to exit, then wait for it to finish.
        mRunning.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lk(mReqMutex);
            mMapRequest.ready = true;   // unblock the wait() in mapperLoop
        }
        mReqCv.notify_one();
        if (mMapperThread.joinable()) mMapperThread.join();
    }

// ─────────────────────────────────────────────────────────────────────────────
//  projectPoint
//
//  Transforms a world-frame 3D point into image pixel coordinates using the
//  given camera pose.  Returns false if the point is behind the camera or
//  falls outside a generous margin around the image boundary.
// ─────────────────────────────────────────────────────────────────────────────

    bool PTAMSystem::projectPoint(const Pose3D& pose, const float pt3[3],
                                  float& u, float& v) const {
        // pc = R * pw + t
        const float* R = pose.R;
        const float  xc = R[0]*pt3[0] + R[1]*pt3[1] + R[2]*pt3[2] + pose.t[0];
        const float  yc = R[3]*pt3[0] + R[4]*pt3[1] + R[5]*pt3[2] + pose.t[1];
        const float  zc = R[6]*pt3[0] + R[7]*pt3[1] + R[8]*pt3[2] + pose.t[2];

        if (zc < 0.01f) return false;

        const float invZ = 1.0f / zc;
        u = mFx * xc * invZ + mCx;
        v = mFy * yc * invZ + mCy;

        // 20px margin accepts points near the border whose features may be
        // tracked even when their origin is slightly outside the nominal frame.
        return u >= -20.f && u < mWidth + 20.f &&
               v >= -20.f && v < mHeight + 20.f;
    }

// ─────────────────────────────────────────────────────────────────────────────
//  findNearestFlow
//
//  Searches the flow vector list for the nearest origin (x0, y0) to the
//  given pixel coordinate, within kMatchRadiusPx.
// ─────────────────────────────────────────────────────────────────────────────

    bool PTAMSystem::findNearestFlow(const FlowResult& flow, float u, float v,
                                     int& outIdx) const {
        const float r2    = kMatchRadiusPx * kMatchRadiusPx;
        float bestDist2   = r2 + 1.0f;
        outIdx = -1;

        for (int i = 0, n = static_cast<int>(flow.vectors.size()); i < n; ++i) {
            const float dx = flow.vectors[i].x0 - u;
            const float dy = flow.vectors[i].y0 - v;
            const float d2 = dx*dx + dy*dy;
            if (d2 < bestDist2) { bestDist2 = d2; outIdx = i; }
        }
        return outIdx >= 0;
    }

// ─────────────────────────────────────────────────────────────────────────────
//  runPnP
//
//  Builds 3D→2D correspondences by projecting each active map point into the
//  current pose, then finding the nearest flow vector origin.  This gives us
//  "I know point P is at 3D position X in the world; in this frame its pixel
//  position is Y" — exactly what solvePnPRansac needs.
//
//  EPNP is chosen over ITERATIVE because it is non-iterative (O(n) once the
//  four eigenvectors are found) and handles more than 4 correspondences well.
//  On Helio G80 with 20–50 correspondences, EPNP finishes in < 2 ms.
// ─────────────────────────────────────────────────────────────────────────────

    Pose3D PTAMSystem::runPnP(const FlowResult& flow) {
        mPnpObj.clear();
        mPnpImg.clear();

        {
            std::lock_guard<std::mutex> lock(mMapMutex);
            for (int i = 0; i < mMapSize; ++i) {
                const MapPoint3D& mp = mMap[i];
                if (!mp.active || mp.confidence < 0.1f) continue;

                float u, v;
                if (!projectPoint(mCurrentPose, mp.pos, u, v)) continue;

                int idx;
                if (!findNearestFlow(flow, u, v, idx)) continue;

                mPnpObj.push_back({ mp.pos[0], mp.pos[1], mp.pos[2] });
                mPnpImg.push_back({ flow.vectors[idx].x0, flow.vectors[idx].y0 });
            }
        }

        if (static_cast<int>(mPnpObj.size()) < kMinPnPPts) {
            return Pose3D{};   // valid = false → caller keeps last pose
        }

        const cv::Mat K = (cv::Mat_<double>(3,3) <<
        mFx, 0.0, mCx,
        0.0, mFy, mCy,
        0.0, 0.0, 1.0);

        // Use the previous pose as initial guess when we have one.  This helps
        // RANSAC converge in fewer iterations on slow, smooth camera motion.
        const bool useGuess = mCurrentPose.valid;
        if (useGuess) {
            cv::Mat Rmat(3, 3, CV_64F);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    Rmat.at<double>(r,c) = mCurrentPose.R[r*3+c];
            cv::Rodrigues(Rmat, mRvec);
            mTvec = (cv::Mat_<double>(3,1) <<
            mCurrentPose.t[0], mCurrentPose.t[1], mCurrentPose.t[2]);
        } else {
            mRvec = cv::Mat::zeros(3, 1, CV_64F);
            mTvec = cv::Mat::zeros(3, 1, CV_64F);
        }

        cv::Mat rvec = mRvec.clone(), tvec = mTvec.clone();
        const bool ok = cv::solvePnPRansac(
                mPnpObj, mPnpImg, K, cv::noArray(),
                rvec, tvec,
                useGuess,
                /*iterationsCount=*/50,
                kPnPReprojPx,
                /*confidence=*/0.99,
                mInliers,
                cv::SOLVEPNP_EPNP
        );

        if (!ok) {
            LOGW("solvePnPRansac failed (%d pts) — keeping last pose",
                 (int)mPnpObj.size());
            return Pose3D{};
        }

        cv::Mat Rout;
        cv::Rodrigues(rvec, Rout);

        Pose3D result{};
        result.valid = true;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                result.R[r*3+c] = static_cast<float>(Rout.at<double>(r,c));
        result.t[0] = static_cast<float>(tvec.at<double>(0));
        result.t[1] = static_cast<float>(tvec.at<double>(1));
        result.t[2] = static_cast<float>(tvec.at<double>(2));

        const int nInliers = mInliers.empty() ? 0 : cv::countNonZero(mInliers);
        LOGI("PnP: %d/%d inliers  t=(%.2f %.2f %.2f)",
             nInliers, (int)mPnpObj.size(),
             result.t[0], result.t[1], result.t[2]);

        return result;
    }

// ─────────────────────────────────────────────────────────────────────────────
//  shouldAddKeyFrame
//
//  A frame qualifies as a new keyframe when:
//    1. It has enough tracked features to produce good triangulations.
//    2. Enough frames have passed since the last keyframe.
//    3. The camera has translated enough to give a wide baseline
//       (narrow baseline → poorly conditioned triangulation).
// ─────────────────────────────────────────────────────────────────────────────

    bool PTAMSystem::shouldAddKeyFrame(const FlowResult& flow) const {
        if (!mCurrentPose.valid) return false;
        if (static_cast<int>(flow.vectors.size()) < kMinFeaturesForKF) return false;

        const int lastKFFrame = (mKfCount > 0)
                                ? mKeyFrames[(mKfCount - 1) % kMaxKeyFrames].frameIdx
                                : -kMinKfGapFrames;
        if (mFrameCount - lastKFFrame < kMinKfGapFrames) return false;

        if (mKfCount > 0) {
            const KeyFrame3D& prev = mKeyFrames[(mKfCount - 1) % kMaxKeyFrames];
            const float dx = mCurrentPose.t[0] - prev.pose.t[0];
            const float dy = mCurrentPose.t[1] - prev.pose.t[1];
            const float dz = mCurrentPose.t[2] - prev.pose.t[2];
            if (std::sqrt(dx*dx + dy*dy + dz*dz) < kMinKfBaseline) return false;
        }

        return true;
    }

// ─────────────────────────────────────────────────────────────────────────────
//  submitKeyFrame
//
//  Stores the current frame as a keyframe and hands a copy (plus the previous
//  keyframe copy) to the mapper thread.  The mapper thread gets full ownership
//  of its copies so no locking of mKeyFrames is ever needed in the mapper.
// ─────────────────────────────────────────────────────────────────────────────

    void PTAMSystem::submitKeyFrame(const FlowResult& flow) {
        KeyFrame3D kf{};
        kf.pose        = mCurrentPose;
        kf.frameIdx    = mFrameCount;
        kf.timestampNs = flow.timestampNs;

        const int n = std::min(static_cast<int>(flow.vectors.size()),
                               static_cast<int>(kMaxObs));
        kf.obsCount = n;
        for (int i = 0; i < n; ++i) {
            kf.pts2D[i*2 + 0] = flow.vectors[i].x0;
            kf.pts2D[i*2 + 1] = flow.vectors[i].y0;
            kf.mpIdx[i]        = -1;
        }

        {
            std::lock_guard<std::mutex> lk(mReqMutex);
            mMapRequest.curr    = kf;
            mMapRequest.hasPrev = (mKfCount > 0);
            if (mKfCount > 0)
                mMapRequest.prev = mKeyFrames[(mKfCount - 1) % kMaxKeyFrames];
            mMapRequest.ready = true;
        }

        // Store in the camera-thread-only ring buffer AFTER copying to the request
        // so the mapper gets the version before this frame is overwritten.
        mKeyFrames[mKfCount % kMaxKeyFrames] = kf;
        ++mKfCount;

        mReqCv.notify_one();
        LOGI("Keyframe %d submitted (%d features, t=(%.2f %.2f %.2f))",
             mKfCount, n, kf.pose.t[0], kf.pose.t[1], kf.pose.t[2]);
    }

// ─────────────────────────────────────────────────────────────────────────────
//  processFrame  — tracker entry point (camera executor thread)
//
//  1. Copy the IMU rotation into the pose (we trust IMU for orientation).
//  2. Attempt PnP to find the translation that minimises reprojection error
//     against the 3D map.  Keep the IMU rotation even if PnP succeeds,
//     because the IMU has lower noise for orientation than PnP.
//  3. Decay map confidence.
//  4. Consider adding a new keyframe.
// ─────────────────────────────────────────────────────────────────────────────

    Pose3D PTAMSystem::processFrame(const FlowResult& flow, const float imuR[9]) {
        ++mFrameCount;

        // Always use IMU rotation — it is more reliable than the PnP rotation for
        // small baselines.  PnP solves only for the translation component.
        for (int i = 0; i < 9; ++i) mCurrentPose.R[i] = imuR[i];

        const bool haveEnoughMap = [this] {
            std::lock_guard<std::mutex> lk(mMapMutex);
            return mMapSize >= kMinPnPPts;
        }();

        if (haveEnoughMap && !flow.vectors.empty()) {
            const Pose3D pnpResult = runPnP(flow);
            if (pnpResult.valid) {
                // Update only translation — rotation comes from IMU.
                mCurrentPose.t[0] = pnpResult.t[0];
                mCurrentPose.t[1] = pnpResult.t[1];
                mCurrentPose.t[2] = pnpResult.t[2];
                mCurrentPose.valid = true;
            }
        } else {
            // Before the map is built, the translation estimate is zero
            // (we have no stereo baseline to infer depth).
            mCurrentPose.t[0] = mCurrentPose.t[1] = mCurrentPose.t[2] = 0.0f;
            mCurrentPose.valid = (mFrameCount > 1);
        }

        // Decay map confidence (brief lock).
        {
            std::lock_guard<std::mutex> lk(mMapMutex);
            for (int i = 0; i < mMapSize; ++i) {
                if (!mMap[i].active) continue;
                mMap[i].confidence *= kDecayFactor;
                if (mMap[i].confidence < kMinConfForKeep) mMap[i].active = false;
            }
        }

        if (shouldAddKeyFrame(flow)) submitKeyFrame(flow);

        return mCurrentPose;
    }

// ─────────────────────────────────────────────────────────────────────────────
//  reprojError  — helper used during triangulation quality check
// ─────────────────────────────────────────────────────────────────────────────

    bool PTAMSystem::reprojError(const float pt3[3], const Pose3D& pose,
                                 float u2d, float v2d, float& errOut) const {
        float u, v;
        if (!projectPoint(pose, pt3, u, v)) { errOut = 1e6f; return false; }
        const float du = u - u2d;
        const float dv = v - v2d;
        errOut = std::sqrt(du*du + dv*dv);
        return true;
    }

// ─────────────────────────────────────────────────────────────────────────────
//  insertMapPoint  — finds a free slot and inserts a new point
// ─────────────────────────────────────────────────────────────────────────────

    int PTAMSystem::insertMapPoint(const float pos[3]) {
        for (int i = 0; i < kMaxMapPoints; ++i) {
            if (!mMap[i].active) {
                mMap[i].pos[0]       = pos[0];
                mMap[i].pos[1]       = pos[1];
                mMap[i].pos[2]       = pos[2];
                mMap[i].confidence   = 0.9f;
                mMap[i].lastSeenFrame = mFrameCount;
                mMap[i].active       = true;
                if (i >= mMapSize) mMapSize = i + 1;
                return i;
            }
        }
        return -1;   // map full
    }

// ─────────────────────────────────────────────────────────────────────────────
//  triangulateAndAdd  — mapper thread
//
//  Matches features between two keyframes by nearest-neighbour search in
//  pixel space (valid when the camera moves slowly relative to feature spacing).
//
//  Then calls cv::triangulatePoints with the normalised projection matrices
//  (K factored out), and filters the output points by:
//    • positive depth in both camera frames
//    • reprojection error < kMaxReprojPx in both keyframes
//    • no near-duplicate already in the map
//
//  The projection matrices are:
//    P_norm = [R | t]   (no K prefix — inputs are already normalised)
//  so cv::triangulatePoints returns points in the camera-convention world frame.
// ─────────────────────────────────────────────────────────────────────────────

    void PTAMSystem::triangulateAndAdd(const KeyFrame3D& kfA, const KeyFrame3D& kfB) {
        // Match features by nearest-neighbour (pixel space).
        const float r2 = kMatchRadForTri * kMatchRadForTri;

        std::vector<cv::Point2f> ptsA, ptsB;
        ptsA.reserve(kfA.obsCount);
        ptsB.reserve(kfA.obsCount);

        for (int i = 0; i < kfA.obsCount; ++i) {
            const float ax = kfA.pts2D[i*2],   ay = kfA.pts2D[i*2+1];
            float bestD2 = r2 + 1.f;
            int   bestJ  = -1;
            for (int j = 0; j < kfB.obsCount; ++j) {
                const float dx = kfB.pts2D[j*2]   - ax;
                const float dy = kfB.pts2D[j*2+1] - ay;
                const float d2 = dx*dx + dy*dy;
                if (d2 < bestD2) { bestD2 = d2; bestJ = j; }
            }
            if (bestJ < 0) continue;
            // Normalise coordinates (remove K) so triangulation operates in the
            // calibrated, distortion-free camera frame.
            ptsA.push_back({ (ax - mCx) / mFx, (ay - mCy) / mFy });
            ptsB.push_back({ (kfB.pts2D[bestJ*2] - mCx) / mFx,
                             (kfB.pts2D[bestJ*2+1] - mCy) / mFy });
        }

        if ((int)ptsA.size() < 4) {
            LOGI("Triangulate: only %d matches — skipping", (int)ptsA.size());
            return;
        }

        // Build 2×N matrices from the matched points.
        cv::Mat pts1(2, (int)ptsA.size(), CV_64F);
        cv::Mat pts2(2, (int)ptsB.size(), CV_64F);
        for (int i = 0; i < (int)ptsA.size(); ++i) {
            pts1.at<double>(0,i) = ptsA[i].x;
            pts1.at<double>(1,i) = ptsA[i].y;
            pts2.at<double>(0,i) = ptsB[i].x;
            pts2.at<double>(1,i) = ptsB[i].y;
        }

        // Normalised projection matrices [R | t] — K is already divided out above.
        auto makeP = [](const KeyFrame3D& kf) {
            cv::Mat P = (cv::Mat_<double>(3,4) <<
                                               kf.pose.R[0], kf.pose.R[1], kf.pose.R[2], kf.pose.t[0],
                    kf.pose.R[3], kf.pose.R[4], kf.pose.R[5], kf.pose.t[1],
                    kf.pose.R[6], kf.pose.R[7], kf.pose.R[8], kf.pose.t[2]
            );
            return P;
        };

        cv::Mat pts4D;
        cv::triangulatePoints(makeP(kfA), makeP(kfB), pts1, pts2, pts4D);

        int added = 0;
        std::lock_guard<std::mutex> lock(mMapMutex);

        for (int i = 0; i < pts4D.cols; ++i) {
            const double w = pts4D.at<double>(3, i);
            if (std::abs(w) < 1e-7) continue;

            const float pos[3] = {
                    static_cast<float>(pts4D.at<double>(0,i) / w),
                    static_cast<float>(pts4D.at<double>(1,i) / w),
                    static_cast<float>(pts4D.at<double>(2,i) / w)
            };

            // Depth in kfA's camera frame (Z component of R*pw + t).
            const auto zInFrame = [&](const KeyFrame3D& kf) {
                const float* R = kf.pose.R;
                return R[6]*pos[0] + R[7]*pos[1] + R[8]*pos[2] + kf.pose.t[2];
            };
            if (zInFrame(kfA) < kMinDepth || zInFrame(kfA) > kMaxDepth) continue;
            if (zInFrame(kfB) < kMinDepth || zInFrame(kfB) > kMaxDepth) continue;

            // Reprojection error in both keyframes (check against the un-normalised
            // pixel coordinates, so we use the original ptsA/ptsB scaled back by K).
            float eA, eB;
            reprojError(pos, kfA.pose,
                        ptsA[i].x * mFx + mCx, ptsA[i].y * mFy + mCy, eA);
            reprojError(pos, kfB.pose,
                        ptsB[i].x * mFx + mCx, ptsB[i].y * mFy + mCy, eB);
            if (eA > kMaxReprojPx || eB > kMaxReprojPx) continue;

            // Duplicate suppression.
            bool dup = false;
            for (int j = 0; j < mMapSize && !dup; ++j) {
                if (!mMap[j].active) continue;
                const float dx = mMap[j].pos[0] - pos[0];
                const float dy = mMap[j].pos[1] - pos[1];
                const float dz = mMap[j].pos[2] - pos[2];
                dup = (dx*dx + dy*dy + dz*dz) < kDuplicateRadius2;
            }
            if (dup) continue;

            if (insertMapPoint(pos) >= 0) ++added;
        }

        LOGI("Triangulated: +%d pts (map size %d)", added, mMapSize);
    }

// ─────────────────────────────────────────────────────────────────────────────
//  mapperLoop  — background thread
// ─────────────────────────────────────────────────────────────────────────────

    void PTAMSystem::mapperLoop() {
        LOGI("Mapper thread started");

        while (mRunning.load(std::memory_order_acquire)) {
            MapRequest req;

            {
                std::unique_lock<std::mutex> lk(mReqMutex);
                mReqCv.wait(lk, [this] {
                    return mMapRequest.ready ||
                           !mRunning.load(std::memory_order_relaxed);
                });
                if (!mRunning.load(std::memory_order_relaxed)) break;
                if (!mMapRequest.ready) continue;
                req               = mMapRequest;   // full copy — camera thread is free
                mMapRequest.ready = false;
            }

            if (req.hasPrev) {
                // Check baseline before paying triangulation cost.
                const float dx = req.curr.pose.t[0] - req.prev.pose.t[0];
                const float dy = req.curr.pose.t[1] - req.prev.pose.t[1];
                const float dz = req.curr.pose.t[2] - req.prev.pose.t[2];
                if (std::sqrt(dx*dx + dy*dy + dz*dz) >= kMinKfBaseline)
                    triangulateAndAdd(req.prev, req.curr);
            }
        }

        LOGI("Mapper thread exiting");
    }

} // namespace assistivenav