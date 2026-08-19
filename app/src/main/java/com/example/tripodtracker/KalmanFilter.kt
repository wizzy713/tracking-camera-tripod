package com.example.tripodtracker

/**
 * A 1D Kalman Filter for predicting object position based on constant velocity,
 * using a discrete white-noise-acceleration process model.
 */
class KalmanFilter(
    private val measurementNoise: Float = DEFAULT_MEASUREMENT_NOISE,
    private val accelerationNoise: Float = DEFAULT_ACCELERATION_NOISE
) {
    // State variables: [position, velocity]
    private var x = 0f
    private var v = 0f

    // Error covariance matrix: [p_p, p_pv], [p_vp, p_vv]
    private var p_p = 1f
    private var p_pv = 0f
    private var p_vp = 0f
    private var p_vv = 1f

    private var lastTimestampNanos = 0L
    private var initialized = false

    val position: Float get() = x
    val velocity: Float get() = v
    val hasEstimate: Boolean get() = initialized

    var lastDt: Float = 0f
        private set

    /**
     * Updates the filter with a new position measurement.
     * @param measurement The raw coordinate from the detector, in pixels.
     * @param timestampNanos A monotonic frame-capture timestamp in nanoseconds
     *   (e.g. ImageProxy.getImageInfo().getTimestamp()), not wall-clock time.
     */
    fun update(measurement: Float, timestampNanos: Long): Float {
        if (!initialized) {
            x = measurement
            v = 0f
            lastTimestampNanos = timestampNanos
            initialized = true
            lastDt = 0f
            return x
        }

        val dt = (timestampNanos - lastTimestampNanos) / 1_000_000_000f
        lastTimestampNanos = timestampNanos

        if (dt <= 0f) {
            // Out-of-order or duplicate frame timestamp; nothing to predict.
            return x
        }
        lastDt = dt

        // 1. Predict state
        x += v * dt

        // 2. Predict covariance: F P F^T + Q, discrete white-noise-acceleration Q.
        val sigmaA2 = accelerationNoise * accelerationNoise
        val dt2 = dt * dt
        val dt3 = dt2 * dt
        val dt4 = dt3 * dt
        val qPp = sigmaA2 * dt4 / 4f
        val qPv = sigmaA2 * dt3 / 2f
        val qVv = sigmaA2 * dt2

        p_p += dt * (dt * p_vv + p_pv + p_vp) + qPp
        p_pv += dt * p_vv + qPv
        p_vp += dt * p_vv + qPv
        p_vv += qVv

        // 3. Update (Correction)
        val y = measurement - x // Innovation
        val s = p_p + measurementNoise // Innovation covariance
        val k_p = p_p / s
        val k_v = p_vp / s

        x += k_p * y
        v += k_v * y

        // Compute from pre-update covariance values -- p_vp/p_vv depend on the
        // old p_p/p_pv, so they must not read values p_p/p_pv already overwrote.
        val newPp = p_p * (1 - k_p)
        val newPpv = p_pv * (1 - k_p)
        val newPvp = p_vp - k_v * p_p
        val newPvv = p_vv - k_v * p_pv

        p_p = newPp
        p_pv = newPpv
        p_vp = newPvp
        p_vv = newPvv

        return x
    }

    /**
     * Predicts the position at a future time.
     * @param futureDtSeconds How many seconds into the future to predict.
     */
    fun predictFuture(futureDtSeconds: Float): Float {
        return x + v * futureDtSeconds
    }

    fun reset() {
        initialized = false
        x = 0f
        v = 0f
        p_p = 1f
        p_pv = 0f
        p_vp = 0f
        p_vv = 1f
        lastDt = 0f
    }

    // Exposed for KalmanFilterTest -- same-module `internal` visibility, not a public API.
    internal val covariancePP: Float get() = p_p
    internal val covariancePV: Float get() = p_pv
    internal val covarianceVP: Float get() = p_vp
    internal val covarianceVV: Float get() = p_vv

    companion object {
        // ML Kit bounding-box centre jitter on a person-sized box is roughly
        // 5-15 px stddev, so R = sigma^2 is roughly 25-225. 100 is the midpoint;
        // tune it from logged RawX/RawY vs FilteredX/FilteredY once real data exists.
        const val DEFAULT_MEASUREMENT_NOISE = 100f

        // Expected magnitude of subject acceleration, in px/s^2. This is a
        // starting point, not a measured value -- tune it from the logged
        // VelocityX/VelocityY columns (see LogManager) before citing results.
        const val DEFAULT_ACCELERATION_NOISE = 80f
    }
}
