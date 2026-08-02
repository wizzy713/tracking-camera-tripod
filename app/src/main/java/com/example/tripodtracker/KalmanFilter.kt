package com.example.tripodtracker

/**
 * A simple 1D Kalman Filter for predicting object position based on constant velocity.
 */
class KalmanFilter {
    // State variables: [position, velocity]
    private var x = 0f
    private var v = 0f

    // Error covariance matrix: [p_p, p_pv], [p_vp, p_vv]
    private var p_p = 1f
    private var p_pv = 0f
    private var p_vp = 0f
    private var p_vv = 1f

    // Process noise covariance (tweak these to adjust responsiveness vs smoothness)
    private val q_p = 0.01f
    private val q_v = 0.01f

    // Measurement noise covariance (uncertainty in ML Kit's bounding box)
    private val r = 0.1f

    private var lastTimestamp = 0L
    private var initialized = false

    /**
     * Updates the filter with a new position measurement.
     * @param measurement The raw X coordinate from the detector.
     * @param timestamp The current system time in milliseconds.
     */
    fun update(measurement: Float, timestamp: Long): Float {
        if (!initialized) {
            x = measurement
            v = 0f
            lastTimestamp = timestamp
            initialized = true
            return x
        }

        val dt = (timestamp - lastTimestamp) / 1000f // Convert to seconds
        lastTimestamp = timestamp

        // 1. Predict state
        x += v * dt
        // v = v (constant velocity assumption)

        // 2. Predict covariance
        p_p += dt * (dt * p_vv + p_pv + p_vp) + q_p
        p_pv += dt * p_vv
        p_vp += dt * p_vv
        p_vv += q_v

        // 3. Update (Correction)
        val y = measurement - x // Innovation
        val s = p_p + r // Innovation covariance
        val k_p = p_p / s // Kalman gain for position
        val k_v = p_vp / s // Kalman gain for velocity

        x += k_p * y
        v += k_v * y

        p_p *= (1 - k_p)
        p_pv *= (1 - k_p)
        p_vp -= k_v * p_p
        p_vv -= k_v * p_pv

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
    }
}
