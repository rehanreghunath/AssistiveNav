#include "AudioEngine.h"
#include <android/log.h>
#include <algorithm>
#include <cmath>
#include <random>

#define LOG_TAG "AudioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace assistivenav {

    AudioEngine::AudioEngine()
            : mDevice(nullptr), mContext(nullptr), mNoiseBuffer(0),
              mSources{}, mSmoothedGain{}, mSmoothedPitch{},
              mSmoothedX{}, mSmoothedY{}, mSmoothedZ{},
              mReady(false), mDiagFrames(0)
    {
        for (float& p : mSmoothedPitch) p = 1.0f;
        initOpenAL();
        if (mReady) { generateNoiseBuffer(); initSources(); }
    }

    AudioEngine::~AudioEngine() { shutdownOpenAL(); }

    void AudioEngine::initOpenAL() {
        setenv("ALSOFT_ANDROID_STREAM_TYPE", "3", 1);
        mDevice = alcOpenDevice(nullptr);
        if (!mDevice) { LOGE("alcOpenDevice failed (0x%x)", alcGetError(nullptr)); return; }

        if (alcIsExtensionPresent(mDevice, "ALC_SOFT_HRTF")) {
            const ALCint attrs[] = { ALC_HRTF_SOFT, ALC_TRUE, ALC_FREQUENCY, kSampleRate, 0 };
            mContext = alcCreateContext(mDevice, attrs);
            LOGI("HRTF context requested at %d Hz", kSampleRate);
        } else {
            const ALCint attrs[] = { ALC_FREQUENCY, kSampleRate, 0 };
            mContext = alcCreateContext(mDevice, attrs);
            LOGI("ALC_SOFT_HRTF unavailable — stereo panning only");
        }

        if (!mContext) {
            LOGE("alcCreateContext failed"); alcCloseDevice(mDevice); mDevice = nullptr; return;
        }
        if (alcMakeContextCurrent(mContext) == ALC_FALSE) {
            LOGE("alcMakeContextCurrent failed");
            alcDestroyContext(mContext); alcCloseDevice(mDevice);
            mContext = nullptr; mDevice = nullptr; return;
        }

        ALCint hrtfStatus = 0;
        alcGetIntegerv(mDevice, ALC_HRTF_STATUS_SOFT, 1, &hrtfStatus);
        LOGI("HRTF status: %d (%s)", hrtfStatus, hrtfStatus==1 ? "ENABLED" : "NOT ENABLED");

        const ALfloat orientation[6] = { 0.f,0.f,-1.f, 0.f,1.f,0.f };
        alListener3f(AL_POSITION, 0.f, 0.f, 0.f);
        alListener3f(AL_VELOCITY, 0.f, 0.f, 0.f);
        alListenerfv(AL_ORIENTATION, orientation);
        alListenerf (AL_GAIN, 1.0f);
        alDistanceModel(AL_NONE);
        mReady = true;
        LOGI("AudioEngine ready (%d sources)", kNumSources);
    }

    void AudioEngine::generateNoiseBuffer() {
        std::vector<int16_t> samples(kNoiseFrames);
        std::mt19937 rng(0xA55174u);
        std::uniform_real_distribution<float> dist(-1.f, 1.f);
        float prevX = 0.f, prevY = 0.f;
        for (int n = 0; n < kNoiseFrames; ++n) {
            const float x = dist(rng);
            const float y = kHpAlpha * (prevY + x - prevX);
            prevX = x; prevY = y;
            samples[n] = static_cast<int16_t>(std::clamp(y,-1.f,1.f) * 32767.f);
        }
        alGenBuffers(1, &mNoiseBuffer);
        alBufferData(mNoiseBuffer, AL_FORMAT_MONO16, samples.data(),
                     static_cast<ALsizei>(samples.size() * sizeof(int16_t)), kSampleRate);
        LOGI("Noise buffer OK (%d frames @ %d Hz)", kNoiseFrames, kSampleRate);
    }

    void AudioEngine::initSources() {
        alGenSources(kNumSources, mSources.data());
        const ALboolean hasSpatialise = alIsExtensionPresent("AL_SOFT_source_spatialize");

        for (int i = 0; i < kNumSources; ++i) {
            const ALuint src = mSources[i];
            alSourcei (src, AL_BUFFER,         static_cast<ALint>(mNoiseBuffer));
            alSourcei (src, AL_LOOPING,        AL_TRUE);
            alSourcef (src, AL_GAIN,           0.0f);
            alSourcef (src, AL_PITCH,          1.0f);
            alSourcef (src, AL_ROLLOFF_FACTOR, 0.0f);
            alSource3f(src, AL_POSITION,       0.f, 0.f, -kSourceRadius);
            alSource3f(src, AL_VELOCITY,       0.f, 0.f, 0.f);
            if (hasSpatialise) alSourcei(src, AL_SOURCE_SPATIALIZE_SOFT, AL_TRUE);
            mSmoothedGain[i]  = 0.f;
            mSmoothedPitch[i] = 1.f;
            mSmoothedX[i]     = 0.f;
            mSmoothedY[i]     = 0.f;
            mSmoothedZ[i]     = -kSourceRadius;
        }
        alSourcePlayv(kNumSources, mSources.data());
        LOGI("%d sources started silent", kNumSources);
    }

    void AudioEngine::updateFromSources(const AudioSourceDesc sources[kMaxAudioSources],
                                        const int   count,
                                        const float suppressionFactor) {
        if (!mReady) return;

        for (int s = 0; s < kNumSources; ++s) {
            const ALuint src = mSources[s];

            // Target values for this source slot.
            float targetGain  = 0.f;
            float targetPitch = 1.f;
            float targetX     = mSmoothedX[s];
            float targetY     = mSmoothedY[s];
            float targetZ     = mSmoothedZ[s];

            if (s < count && sources[s].active) {
                const AudioSourceDesc& d = sources[s];

                // Gain = confidence mapped to [0,1] * proximity * suppression * master.
                const float confNorm = std::clamp(
                        (d.confidence - kMinConfidence) / (kFullConfidence - kMinConfidence),
                        0.f, 1.f);
                targetGain = confNorm * d.proximity * suppressionFactor * kMasterGain;

                // Convert azimuth/elevation to AL Cartesian on a sphere of radius R.
                const float cosEl = std::cos(d.elevationRad);
                targetX = std::sin(d.azimuthRad)  * cosEl * kSourceRadius;
                targetY = std::sin(d.elevationRad)         * kSourceRadius;
                targetZ = -std::cos(d.azimuthRad) * cosEl * kSourceRadius;

                // Pitch: proximity [0,1] → [kPitchLow, kPitchHigh].
                targetPitch = kPitchLow + d.proximity * (kPitchHigh - kPitchLow);
            }

            // EMA smooth gain.
            mSmoothedGain[s] = kGainAlpha   * targetGain
                               + (1.f - kGainAlpha)  * mSmoothedGain[s];
            alSourcef(src, AL_GAIN, mSmoothedGain[s]);

            // EMA smooth 3D position.
            mSmoothedX[s] = kPosAlpha * targetX + (1.f - kPosAlpha) * mSmoothedX[s];
            mSmoothedY[s] = kPosAlpha * targetY + (1.f - kPosAlpha) * mSmoothedY[s];
            mSmoothedZ[s] = kPosAlpha * targetZ + (1.f - kPosAlpha) * mSmoothedZ[s];
            alSource3f(src, AL_POSITION, mSmoothedX[s], mSmoothedY[s], mSmoothedZ[s]);

            // EMA smooth pitch (only when audible, to avoid pop on first activation).
            if (mSmoothedGain[s] > 0.01f) {
                mSmoothedPitch[s] = kPitchAlpha * targetPitch
                                    + (1.f - kPitchAlpha) * mSmoothedPitch[s];
                alSourcef(src, AL_PITCH, mSmoothedPitch[s]);
            }
        }

        if (++mDiagFrames >= 90) {
            mDiagFrames = 0;
            LOGI("PTAM audio | active=%d supp=%.2f | "
                 "s0 g=%.2f p=%.2f pos=(%.1f,%.1f,%.1f) | "
                 "s1 g=%.2f pos=(%.1f,%.1f,%.1f)",
                 count, suppressionFactor,
                 mSmoothedGain[0], mSmoothedPitch[0],
                 mSmoothedX[0], mSmoothedY[0], mSmoothedZ[0],
                 mSmoothedGain[1],
                 mSmoothedX[1], mSmoothedY[1], mSmoothedZ[1]);
            const ALenum err = alGetError();
            if (err != AL_NO_ERROR) LOGE("OpenAL error: 0x%x", err);
        }
    }

    void AudioEngine::updateFromObstacles(const ObstacleFrame& frame,
                                          const float suppressionFactor) {
        if (!mReady) return;

        // Accumulate dominant left/right gain from obstacle frame.
        float targetGainLR[2]  = { 0.f, 0.f };
        float targetY   [2]    = { mSmoothedY[0], mSmoothedY[1] };
        float targetMag [2]    = { 0.f, 0.f };

        for (int i = 0; i < kMaxObstacles; ++i) {
            const TrackedObstacle& obs = frame.obstacles[i];
            if (!obs.active) continue;

            const float confNorm = std::clamp(
                    (obs.confidenceScore - kMinConfidence) / (kFullConfidence - kMinConfidence),
                    0.f, 1.f);
            const float rawGain = confNorm * suppressionFactor * kMasterGain;
            if (rawGain < 1e-4f) continue;

            const float leftW  = 1.f - obs.normX;
            const float rightW = obs.normX;
            const float alY    = 0.5f - obs.normY;   // normY=0 (top) -> +0.5 (head)

            if (leftW  * rawGain > targetGainLR[0]) {
                targetGainLR[0] = leftW  * rawGain;
                targetY[0]      = alY;
                targetMag[0]    = obs.smoothedMag;
            }
            if (rightW * rawGain > targetGainLR[1]) {
                targetGainLR[1] = rightW * rawGain;
                targetY[1]      = alY;
                targetMag[1]    = obs.smoothedMag;
            }
        }

        const float fixedX[2] = { -1.f, 1.f };

        for (int s = 0; s < 2; ++s) {
            const ALuint src = mSources[s];

            mSmoothedGain[s] = kGainAlpha      * targetGainLR[s]
                               + (1.f-kGainAlpha) * mSmoothedGain[s];
            alSourcef(src, AL_GAIN, mSmoothedGain[s]);

            mSmoothedY[s] = kFallbackYAlpha    * targetY[s]
                            + (1.f-kFallbackYAlpha) * mSmoothedY[s];
            alSource3f(src, AL_POSITION, fixedX[s], mSmoothedY[s], kFallbackDepth);

            if (mSmoothedGain[s] > 0.01f) {
                const float t = std::clamp(
                        (targetMag[s] - kFallbackMagLow) / (kFallbackMagHigh - kFallbackMagLow),
                        0.f, 1.f);
                const float tPitch = kPitchLow + t * (kPitchHigh - kPitchLow);
                mSmoothedPitch[s] = kPitchAlpha * tPitch + (1.f-kPitchAlpha) * mSmoothedPitch[s];
                alSourcef(src, AL_PITCH, mSmoothedPitch[s]);
            }
        }

        // Sources 2 and 3 are muted in fallback mode
        for (int s = 2; s < kNumSources; ++s) {
            mSmoothedGain[s] = kGainAlpha * 0.f + (1.f-kGainAlpha) * mSmoothedGain[s];
            alSourcef(mSources[s], AL_GAIN, mSmoothedGain[s]);
        }
    }

    void AudioEngine::shutdownOpenAL() {
        if (!mReady) return;
        mReady = false;
        alSourceStopv(kNumSources, mSources.data());
        alDeleteSources(kNumSources, mSources.data());
        if (mNoiseBuffer) { alDeleteBuffers(1, &mNoiseBuffer); mNoiseBuffer = 0; }
        alcMakeContextCurrent(nullptr);
        if (mContext) { alcDestroyContext(mContext); mContext = nullptr; }
        if (mDevice)  { alcCloseDevice(mDevice);    mDevice  = nullptr; }
        LOGI("AudioEngine shut down");
    }

} // namespace assistivenav