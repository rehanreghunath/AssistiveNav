#pragma once

#include "PTAMTypes.h"
#include "VoxelMap.h"

namespace assistivenav {

// ─────────────────────────────────────────────────────────────────────────────
// SoundMapper
//
// Converts VoxelMap clusters into AudioSourceDesc objects that AudioEngine can
// play directly.
//
// Coordinate transform:
//   Each cluster centroid is in the PTAM world frame.  To produce audio angles
//   that make sense to the user, we must express it in the CURRENT camera frame:
//
//     pc = R * pw + t          (world-to-camera transform from current pose)
//
//   From the camera-frame position:
//     azimuth   = atan2(pc.x,  pc.z)   right-positive, forward = 0
//     elevation = atan2(-pc.y, pc.z)   up-positive    (camera Y+ = down)
//     proximity = exp(-depth / kProxDecay) ∈ (0, 1]
//
//   Only clusters with positive depth (in front of the camera) are included.
//
// Why not use flow direction for azimuth?
//   Flow direction inverts when the user turns — background objects produce
//   rightward flow during a left turn.  The cluster's pixel/3D position in
//   the camera frame is immune to this because it is derived from the
//   triangulated world position, not from the flow vector's direction.
// ─────────────────────────────────────────────────────────────────────────────

    class SoundMapper {
    public:
        SoundMapper() = default;

        /** Convert voxel clusters to audio source descriptors.
         *  pose       — current camera pose (world→camera, from PTAMSystem).
         *  clusters   — up to VoxelMap::kMaxClusters clusters from VoxelMap::getClusters.
         *  count      — number of valid entries in clusters[].
         *  outSources — caller-allocated array of kMaxAudioSources.
         *  Returns the number of active sources written. */
        int computeSources(const VoxelMap::Cluster clusters[VoxelMap::kMaxClusters],
                           int                     count,
                           const Pose3D&           pose,
                           AudioSourceDesc         outSources[kMaxAudioSources]) const;

    private:
        // Proximity decay constant (PTAM units).
        // proximity = exp(-depth / kProxDecay).
        // At depth = kProxDecay → proximity ≈ 0.37 (moderate urgency).
        // At depth = 0.5 units  → proximity ≈ exp(-0.5/2.0) ≈ 0.78 (high urgency).
        static constexpr float kProxDecay   = 2.0f;

        // Clusters with depth beyond kMaxDepth are too far to be actionable.
        static constexpr float kMaxDepth    = 8.0f;

        // Minimum cluster mass for it to generate a source.
        // Prevents ghost clusters from near-empty voxels from generating audio.
        static constexpr float kMinMass     = 0.5f;

        // Reference mass: clusters at or above this mass get confidence = 1.
        static constexpr float kRefMass     = 6.0f;
    };

} // namespace assistivenav