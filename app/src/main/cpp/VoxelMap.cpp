#include "VoxelMap.h"
#include <android/log.h>
#include <algorithm>
#include <cstring>

#define LOG_TAG "VoxelMap"

namespace assistivenav {

    VoxelMap::VoxelMap() {
        mGrid.fill(0.0f);
    }

// ─────────────────────────────────────────────────────────────────────────────
//  toVoxel
//
//  Converts a world-frame coordinate to grid indices.
//  Returns false (and leaves xi/yi/zi undefined) if the point is outside the
//  grid bounds, so callers can discard out-of-bounds map points without error.
// ─────────────────────────────────────────────────────────────────────────────

    bool VoxelMap::toVoxel(float x, float y, float z,
                           int& xi, int& yi, int& zi) const {
        // Normalise to [0, 1] within the grid extent.
        const float nx = (x + kExtentX) / (2.0f * kExtentX);
        const float ny = (y + kExtentY) / (2.0f * kExtentY);
        const float nz = (z + kExtentZ) / (2.0f * kExtentZ);

        if (nx < 0.f || nx >= 1.f ||
            ny < 0.f || ny >= 1.f ||
            nz < 0.f || nz >= 1.f) return false;

        xi = static_cast<int>(nx * kGridX);
        yi = static_cast<int>(ny * kGridY);
        zi = static_cast<int>(nz * kGridZ);

        // Clamp to valid range (float precision at boundary may push one cell over).
        xi = std::min(xi, kGridX - 1);
        yi = std::min(yi, kGridY - 1);
        zi = std::min(zi, kGridZ - 1);
        return true;
    }

    void VoxelMap::insertPoint(float x, float y, float z, float weight) {
        int xi, yi, zi;
        if (!toVoxel(x, y, z, xi, yi, zi)) return;
        const int idx = toIndex(xi, yi, zi);
        mGrid[idx] = std::min(mGrid[idx] + kIncrement * weight, kMaxOccupancy);
    }

    void VoxelMap::decay(const float factor) {
        for (float& v : mGrid) v *= factor;
    }

    void VoxelMap::clear() {
        mGrid.fill(0.0f);
    }

// ─────────────────────────────────────────────────────────────────────────────
//  getClusters
//
//  6-connected flood fill over threshold voxels.  All working state (visited
//  array + BFS queue) lives on the stack so no heap allocation occurs here.
//
//  Cluster centroid: weighted mean of constituent voxel centres, where the
//  weight is the voxel occupancy.  This attracts the centroid toward the
//  densest part of the cluster, giving a stable anchor point for audio.
//
//  The flood-fill queue is intentionally small (512 entries).  If a single
//  blob exceeds that depth the remaining unvisited cells of the blob start
//  new clusters on subsequent seed iterations — a benign over-segmentation
//  that is preferable to a stack overflow.
// ─────────────────────────────────────────────────────────────────────────────

    int VoxelMap::getClusters(Cluster outClusters[kMaxClusters]) const {
        // visited uses bool (1 byte each) → 16 384 bytes on the stack.
        bool visited[kTotal] = {};

        // BFS queue stored on the stack.  If a blob is larger than kQueueSize
        // it is split — acceptable for the audio use case.
        static constexpr int kQueueSize = 512;
        int queue[kQueueSize];

        // Voxel cell dimensions (PTAM units per voxel).
        const float cellX = (2.0f * kExtentX) / kGridX;
        const float cellY = (2.0f * kExtentY) / kGridY;
        const float cellZ = (2.0f * kExtentZ) / kGridZ;

        int clusterCount = 0;

        for (int seed = 0; seed < kTotal && clusterCount < kMaxClusters; ++seed) {
            if (visited[seed] || mGrid[seed] < kClusterThreshold) continue;

            // BFS from this seed.
            float wSumX = 0.f, wSumY = 0.f, wSumZ = 0.f, wTotal = 0.f;
            int   head = 0, tail = 0;
            queue[tail++ % kQueueSize] = seed;
            visited[seed] = true;

            while (head != tail) {
                const int cur  = queue[head++ % kQueueSize];
                const int sz   = cur / (kGridY * kGridX);
                const int rem  = cur % (kGridY * kGridX);
                const int sy   = rem / kGridX;
                const int sx   = rem % kGridX;
                const float occ = mGrid[cur];

                // World-frame voxel centre.
                const float wx = -kExtentX + (sx + 0.5f) * cellX;
                const float wy = -kExtentY + (sy + 0.5f) * cellY;
                const float wz = -kExtentZ + (sz + 0.5f) * cellZ;

                wSumX  += wx * occ;
                wSumY  += wy * occ;
                wSumZ  += wz * occ;
                wTotal += occ;

                // 6-connected face neighbours.
                const int nbrs[6] = {
                        (sx > 0)          ? cur - 1         : -1,
                        (sx < kGridX - 1) ? cur + 1         : -1,
                        (sy > 0)          ? cur - kGridX     : -1,
                        (sy < kGridY - 1) ? cur + kGridX     : -1,
                        (sz > 0)          ? cur - kGridY * kGridX : -1,
                        (sz < kGridZ - 1) ? cur + kGridY * kGridX : -1,
                };
                for (const int nb : nbrs) {
                    if (nb < 0 || visited[nb] || mGrid[nb] < kClusterThreshold)
                        continue;
                    visited[nb] = true;
                    // If queue is full, stop expanding (cluster will be split).
                    const int nextTail = (tail + 1) % kQueueSize;
                    if (nextTail != head % kQueueSize)
                        queue[tail++ % kQueueSize] = nb;
                }
            }

            if (wTotal < 0.5f) continue;

            Cluster& cl = outClusters[clusterCount++];
            cl.cx     = wSumX / wTotal;
            cl.cy     = wSumY / wTotal;
            cl.cz     = wSumZ / wTotal;
            cl.mass   = wTotal;
            cl.active = true;
        }

        return clusterCount;
    }

} // namespace assistivenav