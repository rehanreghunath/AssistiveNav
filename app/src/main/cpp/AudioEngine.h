#pragma once

#include "FlowTypes.h"
#include "PTAMTypes.h"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <array>
#include <cstdint>

#ifndef AL_SOURCE_SPATIALIZE_SOFT
#define AL_SOURCE_SPATIALIZE_SOFT 0x1214
#endif

namespace assistivenav {

    class AudioEngine {
    public:
        AudioEngine();
        ~AudioEngine();

        AudioEngine(const AudioEngine&)            = delete;
        AudioEngine& operator=(const AudioEngine&) = delete;

        /** PRIMARY path when PTAM has built a 3D map.
         *  sources[]        — from SoundMapper::computeSources().
         *  count            — number of active entries in sources[].
         *  suppressionFactor — [0,1] from ImuFusion rotation rate (unchanged logic). */
        void updateFromSources(const AudioSourceDesc sources[kMaxAudioSources],
                               int   count,
                               float suppressionFactor);

        /** FALLBACK path used before the PTAM map is ready.
         *  Identical semantics to the previous iteration's primary path. */
        void updateFromObstacles(const ObstacleFrame& frame,
                                 float suppressionFactor);

        bool isReady() const { return mReady; }

    private:
        ALCdevice*  mDevice;
        ALCcontext* mContext;
        ALuint      mNoiseBuffer;

        // Expanded to kMaxAudioSources so PTAM can drive up to 4 independent
        // 3D sources (one per obstacle cluster) rather than a fixed left/right pair.
        static constexpr int kNumSources = kMaxAudioSources;
        std::array<ALuint, kNumSources> mSources;
        std::array<float,  kNumSources> mSmoothedGain;
        std::array<float,  kNumSources> mSmoothedPitch;
        std::array<float,  kNumSources> mSmoothedX;
        std::array<float,  kNumSources> mSmoothedY;
        std::array<float,  kNumSources> mSmoothedZ;

        bool mReady;
        int  mDiagFrames;

        static constexpr float kSourceRadius = 1.5f;   // distance from listener

        static constexpr float kMinConfidence  = 0.30f;
        static constexpr float kFullConfidence = 0.75f;
        static constexpr float kMasterGain     = 0.65f;
        static constexpr float kGainAlpha      = 0.18f;

        static constexpr float kPitchLow    = 0.80f;
        static constexpr float kPitchHigh   = 1.40f;
        static constexpr float kPitchAlpha  = 0.15f;

        static constexpr float kPosAlpha    = 0.12f;

        static constexpr int   kSampleRate  = 44100;
        static constexpr int   kNoiseFrames = 44100;
        static constexpr float kHpAlpha     = 0.9715f;

        static constexpr float kFallbackSideX    = 1.0f;
        static constexpr float kFallbackDepth    = -1.5f;
        static constexpr float kFallbackYAlpha   = 0.10f;
        static constexpr float kFallbackMagLow   = 2.0f;
        static constexpr float kFallbackMagHigh  = 12.0f;

        void initOpenAL();
        void generateNoiseBuffer();
        void initSources();
        void shutdownOpenAL();
    };

} // namespace assistivenav