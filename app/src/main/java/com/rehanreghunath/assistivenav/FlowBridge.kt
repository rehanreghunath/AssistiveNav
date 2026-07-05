package com.rehanreghunath.assistivenav

object FlowBridge {
    init {
        System.loadLibrary("assistivenav")
    }

    external fun nativeInit(width: Int, height: Int)

    // ── Issue 5: runtime focal length calibration ──────────────────────────
    /**
     * Override the focal length used for IMU rotation compensation.
     *
     * The default fallback (500 px) is a rough estimate for 640×480 at ~65° HFOV.
     * Providing the real value eliminates the systematic residual flow visible on
     * stationary scenes during head movement.
     *
     * Compute it from [android.hardware.camera2.CameraCharacteristics]:
     *   val fMm   = characteristics.get(LENS_INFO_AVAILABLE_FOCAL_LENGTHS)!![0]
     *   val sizeW = characteristics.get(SENSOR_INFO_PHYSICAL_SIZE)!!.width   // mm
     *   val fPx   = fMm * (imageWidthPx / sizeW)
     *
     * Call once from the main thread after [nativeInit] and before the first
     * camera frame.  Values outside [100, 5000] px are silently ignored.
     *
     * @param focalLengthPx  f_mm * (image_width_px / sensor_width_mm)
     */
    external fun nativeSetFocalLength(focalLengthPx: Float)

    /**
     * Deliver a rotation-vector sensor event to the IMU fusion layer.
     *
     * Call this from your SensorEventListener on every TYPE_ROTATION_VECTOR
     * event.  Pass [event.values] directly -> the array is [x, y, z, w] (unit
     * quaternion) and may optionally carry a 5th element (heading accuracy)
     * which is ignored by the native side.
     *
     * @param quaternion FloatArray of length 4 or 5 from SensorEvent.values
     * @param timestampNs event.timestamp from the SensorEvent
     */
    external fun nativeUpdateImu(quaternion: FloatArray, timestampNs: Long)

    /**
     * Process one camera frame.
     *
     * @param yPlane     Raw bytes of the Y-plane from YUV_420_888
     * @param width      Frame width in pixels
     * @param height     Frame height in pixels
     * @param rowStride  Bytes per row (may be > width due to hardware padding)
     * @param timestampNs Frame timestamp from CameraX in nanoseconds
     * @return Float[3] = [trackedCount, totalPts, globalMeanMag]. Null on error.
     */
    external fun nativeProcessFrame(
        yPlane: ByteArray,
        width: Int,
        height: Int,
        rowStride: Int,
        timestampNs: Long
    ): FloatArray?

    /** Returns the RGBA overlay for the last processed frame, or null if not ready. */
    external fun nativeGetRgbaFrame(): ByteArray?

    external fun nativeGetGridResult(): FloatArray?

    external fun nativeGetPtamPointCount(): Int
    external fun nativeSetRenderRotation(degrees: Int)
    external fun nativeDestroy()
}