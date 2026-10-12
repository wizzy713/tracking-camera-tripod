package com.example.tripodtracker

/**
 * Everything that is tuned differently per detection mode: the tripod's PID
 * gains and slew limit (sent to the firmware as a CFG packet) and the app-side
 * Kalman filter, prediction horizon and hold-still band. One profile is kept per [DetectionMode] and applied
 * whenever that mode is selected, so tuning the ball does not undo the tuning
 * for a person and the other way round.
 */
data class ModeProfile(
    val kp: Float,
    val ki: Float,
    val kd: Float,
    val maxSpeedOffsetUs: Float,
    val deadzone: Float,
    val minOffsetUs: Float,
    val measurementNoise: Float,
    val accelerationNoise: Float,
    val predictionHorizonSeconds: Float,
    /** Hold-still band (see [SettleHold]): stop within [holdEnter], restart beyond [holdExit]. Exit 0 turns it off. */
    val holdEnter: Float,
    val holdExit: Float,
    /**
     * Subject speed, in half-frames per second, above which the hold-still band
     * stays out of the way. Only used with room-coordinate tracking, where the
     * filter's velocity is the subject's own. 0 means the band ignores speed.
     */
    val holdSpeed: Float,
    /** Firmware pulse ramp limit, in us per second. */
    val slewUsPerS: Float
) {
    fun toTripodConfig() = TripodConfig(kp, ki, kd, maxSpeedOffsetUs, deadzone, minOffsetUs, slewUsPerS)

    /** Takes the gains from [c]. The slew limit is not one of them: it stays the profile's own. */
    fun withTripodConfig(c: TripodConfig) = copy(
        kp = c.kp, ki = c.ki, kd = c.kd,
        maxSpeedOffsetUs = c.maxSpeedOffsetUs, deadzone = c.deadzone, minOffsetUs = c.minOffsetUs
    )

    /** `key=value` pairs joined by `;`, for SharedPreferences. */
    fun serialize(): String = listOf(
        "kp" to kp, "ki" to ki, "kd" to kd, "ms" to maxSpeedOffsetUs, "dz" to deadzone, "mo" to minOffsetUs,
        "r" to measurementNoise, "sa" to accelerationNoise, "h" to predictionHorizonSeconds,
        "he" to holdEnter, "hx" to holdExit, "hv" to holdSpeed, "sl" to slewUsPerS
    ).joinToString(";") { (k, v) -> "$k=$v" }

    companion object {
        /**
         * Starting values per mode, for use with room-coordinate tracking
         * (see [RoomFrame]), where the prediction horizon acts as a feed-forward
         * of the subject's own speed.
         *
         * FACE is the configuration that worked on the rig on 2026-10-12 (single
         * runs): the pendulum gains and filter, a horizon of 0.40 s (the value at
         * which the feed-forward cancels the following lag for the measured servo
         * gain), a hold-still band of 0.06 / 0.16 that stopped the hunting at
         * rest, and a slew limit of 2000 us/s.
         *
         * BODY copies FACE with a wider deadzone and band, because the chest
         * point from the pose model jitters more than a face box. It has not
         * been tested.
         *
         * BALL keeps the gains, filter, horizon and slew limit measured on the
         * pendulum on 2026-10-06, which was done in image coordinates. The band
         * and its speed limit are new and untested there: the band is for a ball
         * at rest, and the speed limit keeps it out of the way while the ball
         * swings.
         */
        fun defaultFor(mode: DetectionMode): ModeProfile = when (mode) {
            DetectionMode.BALL -> ModeProfile(
                kp = 18f, ki = 0f, kd = 0f, maxSpeedOffsetUs = 300f, deadzone = 0.03f, minOffsetUs = 52f,
                measurementNoise = 5f, accelerationNoise = 300f, predictionHorizonSeconds = 0.2f,
                holdEnter = 0.03f, holdExit = 0.10f, holdSpeed = 0.3f, slewUsPerS = 1000f
            )
            DetectionMode.FACE -> ModeProfile(
                kp = 18f, ki = 0f, kd = 0f, maxSpeedOffsetUs = 300f, deadzone = 0.06f, minOffsetUs = 52f,
                measurementNoise = 5f, accelerationNoise = 300f, predictionHorizonSeconds = 0.4f,
                holdEnter = 0.06f, holdExit = 0.16f, holdSpeed = 0f, slewUsPerS = 2000f
            )
            DetectionMode.BODY -> ModeProfile(
                kp = 18f, ki = 0f, kd = 0f, maxSpeedOffsetUs = 300f, deadzone = 0.08f, minOffsetUs = 52f,
                measurementNoise = 5f, accelerationNoise = 300f, predictionHorizonSeconds = 0.4f,
                holdEnter = 0.08f, holdExit = 0.18f, holdSpeed = 0f, slewUsPerS = 2000f
            )
        }

        /** Inverse of [serialize]. Missing or unreadable fields keep [fallback]'s value. */
        fun parse(text: String?, fallback: ModeProfile): ModeProfile {
            if (text.isNullOrBlank()) return fallback
            val f = text.split(";").mapNotNull { part ->
                val kv = part.split("=", limit = 2)
                kv.getOrNull(1)?.toFloatOrNull()?.takeIf { it.isFinite() }?.let { kv[0].trim() to it }
            }.toMap()
            return ModeProfile(
                kp = f["kp"] ?: fallback.kp,
                ki = f["ki"] ?: fallback.ki,
                kd = f["kd"] ?: fallback.kd,
                maxSpeedOffsetUs = f["ms"] ?: fallback.maxSpeedOffsetUs,
                deadzone = f["dz"] ?: fallback.deadzone,
                minOffsetUs = f["mo"] ?: fallback.minOffsetUs,
                measurementNoise = f["r"] ?: fallback.measurementNoise,
                accelerationNoise = f["sa"] ?: fallback.accelerationNoise,
                predictionHorizonSeconds = f["h"] ?: fallback.predictionHorizonSeconds,
                holdEnter = f["he"] ?: fallback.holdEnter,
                holdExit = f["hx"] ?: fallback.holdExit,
                holdSpeed = f["hv"] ?: fallback.holdSpeed,
                slewUsPerS = f["sl"] ?: fallback.slewUsPerS
            )
        }
    }
}
