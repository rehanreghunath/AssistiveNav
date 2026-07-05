#pragma once

#include <array>

namespace assistivenav {

// ─────────────────────────────────────────────────────────────────────────────
// VoxelMap
//
// A fixed-size 3D occupancy grid in the PTAM world frame.
//
// Grid layout (symmetric about the world-frame origin):
//   X ∈ [-kExtentX, kExtentX]   (left ↔ right)
//   Y ∈ [-kExtentY, kExtentY]   (up   ↔ down, camera Y+ = down)
//   Z ∈ [-kExtentZ, kExtentZ]   (back ↔ forward)
//
// Each voxel stores an occupancy float ∈ [0, 1].
// insertPoint() increases occupancy; decay() multiplies by a factor < 1 so
// stale voxels fade out as the user moves away from an area.
//
// getClusters() returns the centroids of connected occupied regions via
// a 6-connected (face-neighbour) flood fill.  Up to kMaxClusters clusters
// are reported in descending mass order.
// ─────────────────────────────────────────────────────────────────────────────

    class VoxelMap {
    public:
        static constexpr int   kGridX     = 32;
        static constexpr int   kGridY     = 16;
        static constexpr int   kGridZ     = 32;
        static constexpr int   kTotal     = kGridX * kGridY * kGridZ;  // 16 384

        // Extent of the grid in PTAM units (1 unit ≈ first keyframe baseline).
        // Choose values wide enough to capture nearby obstacles without wasting
        // resolution on areas the user will never navigate.
        static constexpr float kExtentX   = 4.0f;
        static constexpr float kExtentY   = 2.0f;
        static constexpr float kExtentZ   = 6.0f;

        struct Cluster {
            float cx, cy, cz;   // world-frame centroid (PTAM units)
            float mass;          // sum of constituent voxel occupancies
            bool  active;
        };
        static constexpr int kMaxClusters = 6;

        VoxelMap();

        /** Increase the occupancy of the voxel containing world point (x,y,z).
         *  weight scales the increment (default 1.0).
         *  Points outside the grid bounds are silently discarded. */
        void insertPoint(float x, float y, float z, float weight = 1.0f);

        /** Multiply all voxel occupancies by factor (0 < factor < 1). */
        void decay(float factor);

        /** Zero every voxel. */
        void clear();

        /** Run flood fill and return up to kMaxClusters occupied regions.
         *  Returns the number of clusters found. */
        int getClusters(Cluster outClusters[kMaxClusters]) const;

    private:
        std::array<float, kTotal> mGrid;

        static constexpr float kIncrement        = 0.40f;
        static constexpr float kMaxOccupancy     = 1.00f;
        static constexpr float kClusterThreshold = 0.20f;  // voxel active above this

        inline int  toIndex(int xi, int yi, int zi) const {
            return zi * kGridY * kGridX + yi * kGridX + xi;
        }
        bool toVoxel(float x, float y, float z, int& xi, int& yi, int& zi) const;
    };

} // namespace assistivenav