#include "SoundMapper.h"
#include <cmath>
#include <algorithm>

namespace assistivenav {

    int SoundMapper::computeSources(
            const VoxelMap::Cluster clusters[VoxelMap::kMaxClusters],
            int                     count,
            const Pose3D&           pose,
            AudioSourceDesc         outSources[kMaxAudioSources]) const {

        int numOut = 0;

        for (int i = 0; i < count && numOut < kMaxAudioSources; ++i) {
            const VoxelMap::Cluster& cl = clusters[i];
            if (!cl.active || cl.mass < kMinMass) continue;

            // Transform world-frame centroid into current camera frame.
            // pc = R * pw + t
            const float* R  = pose.R;
            const float  xc = R[0]*cl.cx + R[1]*cl.cy + R[2]*cl.cz + pose.t[0];
            const float  yc = R[3]*cl.cx + R[4]*cl.cy + R[5]*cl.cz + pose.t[1];
            const float  zc = R[6]*cl.cx + R[7]*cl.cy + R[8]*cl.cz + pose.t[2];

            // Ignore obstacles behind the camera.
            if (zc <= 0.01f) continue;
            if (zc > kMaxDepth) continue;

            // Azimuth: angle in the horizontal plane from the forward direction.
            // atan2(x, z) gives right-positive azimuth when Z is forward.
            const float azimuth   = std::atan2(xc, zc);

            // Elevation: angle above/below horizontal.
            // Camera Y+ = down, so flip sign for "up-positive" convention.
            const float elevation = std::atan2(-yc, zc);

            // Proximity: exponential fall-off with depth.
            // exp(-0) = 1.0 at zero depth; exp(-4) ≈ 0.02 at kMaxDepth/2.
            const float proximity = std::exp(-zc / kProxDecay);

            // Confidence scales with cluster mass, capped at 1.
            const float confidence = std::min(cl.mass / kRefMass, 1.0f);

            AudioSourceDesc& src = outSources[numOut++];
            src.azimuthRad   = azimuth;
            src.elevationRad = elevation;
            src.proximity    = proximity;
            src.confidence   = confidence;
            src.active       = true;
        }

        return numOut;
    }

} // namespace assistivenav