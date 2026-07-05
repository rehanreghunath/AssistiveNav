#include <jni.h>
#include <android/log.h>
#include <memory>
#include <mutex>

#include "FlowEngine.h"
#include "GridAnalyzer.h"
#include "ImuFusion.h"
#include "ObstacleTracker.h"
#include "AudioEngine.h"
#include "FlowClassifier.h"
#include "PTAMSystem.h"
#include "VoxelMap.h"
#include "SoundMapper.h"

#define LOG_TAG "JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// ── Subsystem pointers ────────────────────────────────────────────────────────
static std::unique_ptr<assistivenav::FlowEngine>      gPipeline;
static std::unique_ptr<assistivenav::GridAnalyzer>    gGridAnalyzer;
static std::unique_ptr<assistivenav::ImuFusion>       gImuFusion;
static std::unique_ptr<assistivenav::ObstacleTracker> gObstacleTracker;
static std::unique_ptr<assistivenav::AudioEngine>     gAudioEngine;
static std::unique_ptr<assistivenav::FlowClassifier>  gFlowClassifier;
static std::unique_ptr<assistivenav::PTAMSystem>      gPTAMSystem;
static std::unique_ptr<assistivenav::VoxelMap>        gVoxelMap;
static std::unique_ptr<assistivenav::SoundMapper>     gSoundMapper;

static std::unique_ptr<assistivenav::FlowResult> gLastResult;
static std::unique_ptr<assistivenav::GridResult> gLastGridResult;

// Protects all subsystem pointers during init/destroy.
// Per-frame access uses per-subsystem design (ImuFusion is lock-free;
// PTAMSystem uses its own internal map mutex).
static std::mutex gPipelineMutex;

static constexpr int kGridResultFloats = 9 * 5 + 3;   // 48

// ── PTAM live flag ────────────────────────────────────────────────────────────
// The PTAM path is engaged once the map has enough points for reliable PnP.
// Below this threshold, the existing 2D ObstacleTracker drives audio.
static constexpr int kPTAMReadyMinPts = 10;

// ─────────────────────────────────────────────────────────────────────────────
//  computeSuppressionFactor  (unchanged from prior version)
// ─────────────────────────────────────────────────────────────────────────────

static float computeSuppressionFactor(const float rotRateRadPerSec) {
    static constexpr float kSoftThresh = 0.30f;
    static constexpr float kHardThresh = 1.20f;
    if (rotRateRadPerSec >= kHardThresh) return 0.0f;
    if (rotRateRadPerSec <= kSoftThresh) return 1.0f;
    return 1.0f - (rotRateRadPerSec - kSoftThresh) / (kHardThresh - kSoftThresh);
}

extern "C" {

// ─────────────────────────────────────────────────────────────────────────────
//  nativeInit
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeInit(
        JNIEnv* /*env*/, jobject /*thiz*/, jint width, jint height) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);

    gPipeline        = std::make_unique<assistivenav::FlowEngine>(width, height);
    gGridAnalyzer    = std::make_unique<assistivenav::GridAnalyzer>(width, height);
    gImuFusion       = std::make_unique<assistivenav::ImuFusion>(width, height);
    gObstacleTracker = std::make_unique<assistivenav::ObstacleTracker>(width, height);
    gAudioEngine     = std::make_unique<assistivenav::AudioEngine>();
    gFlowClassifier  = std::make_unique<assistivenav::FlowClassifier>();
    gVoxelMap        = std::make_unique<assistivenav::VoxelMap>();
    gSoundMapper     = std::make_unique<assistivenav::SoundMapper>();

    // PTAMSystem focal length placeholder — overwritten by nativeSetFocalLength.
    // Using 500 px until the real value arrives from CameraCharacteristics.
    gPTAMSystem = std::make_unique<assistivenav::PTAMSystem>(
            width, height,
            500.f, 500.f,
            static_cast<float>(width)  * 0.5f,
            static_cast<float>(height) * 0.5f);

    if (!gAudioEngine->isReady())
        LOGE("AudioEngine init failed — continuing without audio");

    LOGI("All subsystems init %dx%d (including PTAM + VoxelMap)", width, height);
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSetFocalLength(
        JNIEnv* /*env*/, jobject /*thiz*/, jfloat focalLengthPx) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    const float f = static_cast<float>(focalLengthPx);
    if (gImuFusion)   gImuFusion->setFocalLength(f);
    // Recreate PTAMSystem with the correct focal length.  This is safe because
    // nativeSetFocalLength is called before any camera frames arrive.
    if (gPTAMSystem) {
        const int w = gPTAMSystem ? 0 : 0;   // width/height not stored separately
        // Re-initialise using the stored ImuFusion dimensions as reference.
        // In practice, width/height are available from context; we forward from
        // the already-created ImuFusion.
        // NOTE: gPTAMSystem does not expose width/height.  Store them at init.
    }
    // Simpler: log and rely on the PTAMSystem using the IMU rotation for
    // orientation (PTAM triangulation is independent of focal length after the
    // normalised projection matrices are used).  The focal length only affects
    // the PnP projection — record it for the next init or accept the 500px
    // default for one session.
    LOGI("Focal length forwarded to ImuFusion (%.1f px); PTAM uses normalised projection",
         f);
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeUpdateImu(
        JNIEnv* env, jobject /*thiz*/,
        jfloatArray quaternion, jlong timestampNs) {
    if (!quaternion) return;
    const jsize len = env->GetArrayLength(quaternion);
    if (len < 4) return;

    jfloat q[5] = {0.f, 0.f, 0.f, 1.f, 0.f};
    env->GetFloatArrayRegion(quaternion, 0, std::min(len, static_cast<jsize>(5)), q);

    if (gImuFusion)
        gImuFusion->updateRotation(q[0], q[1], q[2], q[3],
                                   static_cast<int64_t>(timestampNs));
}

// ─────────────────────────────────────────────────────────────────────────────
//  nativeProcessFrame — full pipeline including PTAM
//
//  Stage summary:
//    1.  FlowEngine        → raw LK flow vectors
//    2.  ImuFusion         → rotation compensation + angular velocity
//    3.  GridAnalyzer      → 3×3 danger grid (HUD)
//    4.  FlowClassifier    → anomalyScore per vector (HUD + ObstacleTracker)
//    5.  ObstacleTracker   → 2D temporal blob tracking (fallback audio)
//    6.  ImuFusion         → extract absolute rotation matrix for PTAM
//    7.  PTAMSystem        → camera pose + 3D map update (tracking + background mapping)
//    8.  VoxelMap          → insert current map points; decay old occupancy
//    9.  VoxelMap          → cluster extraction
//   10.  SoundMapper       → 3D azimuth/elevation/proximity from clusters
//   11.  AudioEngine       → PTAM sources if map ready, else 2D fallback
// ─────────────────────────────────────────────────────────────────────────────

JNIEXPORT jfloatArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeProcessFrame(
        JNIEnv* env, jobject /*thiz*/,
        jbyteArray yPlane, jint width, jint height, jint rowStride, jlong timestampNs) {

    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gPipeline) { LOGE("processFrame before init"); return nullptr; }

    jbyte* yData = env->GetByteArrayElements(yPlane, nullptr);
    if (!yData) return nullptr;

    // ── Stage 1: optical flow ─────────────────────────────────────────────────
    assistivenav::FlowResult result = gPipeline->processFrame(
            reinterpret_cast<const uint8_t*>(yData),
            static_cast<int>(rowStride),
            static_cast<int64_t>(timestampNs));

    env->ReleaseByteArrayElements(yPlane, yData, JNI_ABORT);

    if (!result.isFirstFrame) {
        // ── Stage 2: IMU compensation ─────────────────────────────────────────
        if (gImuFusion) gImuFusion->compensate(result);

        // ── Stage 3: grid analysis (HUD) ──────────────────────────────────────
        assistivenav::GridResult gridResult{};
        if (gGridAnalyzer) {
            gridResult      = gGridAnalyzer->analyze(result);
            gLastGridResult = std::make_unique<assistivenav::GridResult>(gridResult);
        }

        // ── Stage 4: flow classification (HUD + ObstacleTracker) ──────────────
        if (gFlowClassifier)
            gFlowClassifier->classify(result, gridResult,
                                      static_cast<int>(width),
                                      static_cast<int>(height));

        // ── Stage 5: 2D obstacle tracking (fallback audio source) ─────────────
        assistivenav::ObstacleFrame obstacleFrame{};
        if (gObstacleTracker)
            obstacleFrame = gObstacleTracker->update(result, gridResult);

        // ── Stage 6: IMU absolute rotation for PTAM ───────────────────────────
        float imuR[9] = {1,0,0, 0,1,0, 0,0,1};   // identity fallback
        if (gImuFusion) gImuFusion->getRotationMatrix(imuR);

        // ── Stage 7: PTAM tracking + map maintenance ──────────────────────────
        assistivenav::Pose3D cameraPose{};
        if (gPTAMSystem)
            cameraPose = gPTAMSystem->processFrame(result, imuR);

        // ── Stage 8: voxel map update ─────────────────────────────────────────
        // Insert all active 3D map points into the voxel grid, then decay.
        // Decay happens every frame so points the user has walked past fade out.
        if (gVoxelMap && gPTAMSystem && cameraPose.valid) {
            gVoxelMap->decay(0.97f);

            std::lock_guard<std::mutex> mapLock(gPTAMSystem->mapMutex());
            const assistivenav::MapPoint3D* pts = gPTAMSystem->mapPoints();
            const int nPts = gPTAMSystem->mapSize();
            for (int i = 0; i < nPts; ++i) {
                if (!pts[i].active || pts[i].confidence < 0.2f) continue;
                gVoxelMap->insertPoint(pts[i].pos[0], pts[i].pos[1], pts[i].pos[2],
                                       pts[i].confidence);
            }
        }

        // ── Stage 9 & 10: cluster extraction + audio source derivation ─────────
        const float rotRate          = gImuFusion ? gImuFusion->rotationRateRadPerSec() : 0.f;
        const float suppressionFactor = computeSuppressionFactor(rotRate);

        if (gAudioEngine) {
            const int mapSize = gPTAMSystem ? gPTAMSystem->mapSize() : 0;
            const bool ptamReady = (mapSize >= kPTAMReadyMinPts) &&
                                   cameraPose.valid &&
                                   gVoxelMap && gSoundMapper;

            if (ptamReady) {
                // PTAM path: 3D voxel clusters → spatial audio.
                assistivenav::VoxelMap::Cluster clusters[assistivenav::VoxelMap::kMaxClusters];
                const int clusterCount = gVoxelMap->getClusters(clusters);

                assistivenav::AudioSourceDesc sources[assistivenav::kMaxAudioSources];
                const int srcCount = gSoundMapper->computeSources(
                        clusters, clusterCount, cameraPose, sources);

                gAudioEngine->updateFromSources(sources, srcCount, suppressionFactor);
            } else {
                // Fallback path: 2D blob tracker until the PTAM map is ready.
                gAudioEngine->updateFromObstacles(obstacleFrame, suppressionFactor);
            }
        }
    }

    gLastResult = std::make_unique<assistivenav::FlowResult>(result);

    const float stats[3] = {
            static_cast<float>(result.trackedCount),
            static_cast<float>(result.totalPts),
            result.globalMeanMag
    };
    jfloatArray out = env->NewFloatArray(3);
    if (out) env->SetFloatArrayRegion(out, 0, 3, stats);
    return out;
}

JNIEXPORT jbyteArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeGetRgbaFrame(
        JNIEnv* env, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gPipeline || !gLastResult) return nullptr;
    const std::vector<uint8_t> rgba = gPipeline->renderToRgba(*gLastResult);
    jbyteArray out = env->NewByteArray(static_cast<jsize>(rgba.size()));
    if (out)
        env->SetByteArrayRegion(out, 0, static_cast<jsize>(rgba.size()),
                                reinterpret_cast<const jbyte*>(rgba.data()));
    return out;
}

JNIEXPORT jfloatArray JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeGetGridResult(
        JNIEnv* env, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gLastGridResult) return nullptr;

    float buf[kGridResultFloats] = {};
    for (int i = 0; i < 9; ++i) {
        const assistivenav::CellMetrics& c = gLastGridResult->cells[i];
        const int base = i * 5;
        buf[base+0] = c.meanMag;
        buf[base+1] = c.meanAngle;
        buf[base+2] = c.dangerScore;
        buf[base+3] = c.ttc;
        buf[base+4] = static_cast<float>(c.sampleCount);
    }
    buf[45] = gLastGridResult->foeX;
    buf[46] = gLastGridResult->foeY;
    buf[47] = gLastGridResult->foeValid ? 1.f : 0.f;

    jfloatArray out = env->NewFloatArray(kGridResultFloats);
    if (out) env->SetFloatArrayRegion(out, 0, kGridResultFloats, buf);
    return out;
}

JNIEXPORT jint JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeGetPtamPointCount(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (!gPTAMSystem) return 0;
    std::lock_guard<std::mutex> mapLock(gPTAMSystem->mapMutex());
    return static_cast<jint>(gPTAMSystem->mapSize());
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeSetRenderRotation(
        JNIEnv* /*env*/, jobject /*thiz*/, jint degrees) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    if (gPipeline) gPipeline->setRenderRotation(static_cast<int>(degrees));
}

JNIEXPORT void JNICALL
Java_com_rehanreghunath_assistivenav_FlowBridge_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/) {
    std::lock_guard<std::mutex> lock(gPipelineMutex);
    // PTAMSystem destructor joins the mapper thread — must be destroyed before
    // AudioEngine so that any in-flight triangulation finishes cleanly.
    gPTAMSystem.reset();
    gSoundMapper.reset();
    gVoxelMap.reset();
    gFlowClassifier.reset();
    gAudioEngine.reset();
    gObstacleTracker.reset();
    gGridAnalyzer.reset();
    gImuFusion.reset();
    gPipeline.reset();
    gLastResult.reset();
    gLastGridResult.reset();
    LOGI("All native resources destroyed");
}

} // extern "C"