#pragma once

#include <array>
#include <cstdint>

namespace assistivenav {

// ── Capacity constants ────────────────────────────────────────────────────────
    static constexpr int kMaxMapPoints = 500;
    static constexpr int kMaxKeyFrames = 8;     // ring buffer — old frames are evicted
    static constexpr int kMaxObs       = 200;   // 2D observations stored per keyframe

// ── Camera pose in the PTAM world frame ───────────────────────────────────────
// Convention: world-to-camera transform.
//   pc = R * pw + t    (transforms a world point into camera coordinates)
//   R is row-major float[9]; t is float[3].
// The world frame is defined by the device orientation at session start
// (ImuFusion records the initial quaternion and removes it from subsequent
// readings, so the first frame has R = identity, t = 0).
    struct Pose3D {
        float R[9] = {1,0,0, 0,1,0, 0,0,1};
        float t[3] = {0,0,0};
        bool  valid = false;
    };

// ── One triangulated 3D point in the world frame ──────────────────────────────
    struct MapPoint3D {
        float pos[3]        = {};
        float confidence    = 0.0f;   // [0,1]; decays with age, resets on re-observation
        int   lastSeenFrame = 0;
        bool  active        = false;
    };

// ── One camera keyframe with its 2D feature observations ──────────────────────
    struct KeyFrame3D {
        Pose3D  pose;
        int     frameIdx    = 0;
        int64_t timestampNs = 0;

        // Interleaved pixel positions: [x0, y0, x1, y1, ...]
        std::array<float, kMaxObs * 2> pts2D = {};
        // Map-point slot index for each observation (-1 = not yet matched)
        std::array<int,   kMaxObs>     mpIdx = {};
        int obsCount = 0;
    };

// ── Audio source descriptor produced by SoundMapper ──────────────────────────
// All angles in radians; listener faces +Z in camera frame.
    struct AudioSourceDesc {
        float azimuthRad;      // horizontal: 0 = ahead, + = right, - = left
        float elevationRad;    // vertical:   0 = level, + = above, - = below
        float proximity;       // [0,1]: 1 = very close, 0 = distant
        float confidence;      // [0,1]: cluster mass normalised
        bool  active;
    };

    static constexpr int kMaxAudioSources = 4;

} // namespace assistivenav