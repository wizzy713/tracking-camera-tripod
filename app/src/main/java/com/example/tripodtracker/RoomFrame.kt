package com.example.tripodtracker

import kotlin.math.atan
import kotlin.math.tan

/**
 * How far the camera has turned, expressed as a shift of the image in pixels,
 * at the moment a frame was captured ([frameX], [frameY]) and at the moment its
 * error is sent ([sendX], [sendY]). Subtracting the frame shift from a detected
 * position gives the subject's position in room coordinates, which does not
 * move when the camera does; adding a shift back returns to the image.
 */
data class CameraShift(val frameX: Float, val frameY: Float, val sendX: Float, val sendY: Float) {
    companion object {
        val NONE = CameraShift(0f, 0f, 0f, 0f)
    }
}

/**
 * Tracks the subject in room coordinates by removing the camera's own rotation,
 * measured by the gyroscope, from each detection.
 *
 * Without this the Kalman filter works in image coordinates, where a camera
 * turning at 30 deg/s makes a subject who is standing still appear to move at
 * 30 deg/s the other way. The filter then reports that motion as the subject's
 * velocity, the prediction horizon extrapolates it, and the loop corrects for
 * movement it caused itself: measured on the rig as a 1 Hz swing of 16 deg with
 * the subject still. In room coordinates the filter's velocity is the subject's
 * own, so the same horizon acts as a feed-forward of the subject's speed.
 */
class RoomFrame(private val imu: ImuReader) {
    /**
     * [OFF], [PAN] or [PAN_TILT]. Pan is on by default, in every detection mode.
     * Tilt stays in image coordinates until its sign has been checked on the rig.
     */
    @Volatile var mode = PAN

    /**
     * Use the camera's angle when the error is sent rather than when the frame
     * was captured, so the error allows for how far the camera turned while
     * the frame was being analysed.
     */
    @Volatile var delayCompensation = true

    /** Horizontal half-angle of the analysis frame's field of view with the front camera, in degrees. */
    @Volatile var halfAngleXDeg = DEFAULT_HALF_ANGLE_X_DEG

    /**
     * The same for the rear camera. Not measured yet: it starts at the front
     * camera's value. A wrong value leaves that share of the camera's own
     * motion in the subject's estimated motion (10% out leaves 10%).
     */
    @Volatile var halfAngleRearXDeg = DEFAULT_HALF_ANGLE_X_DEG

    val active: Boolean get() = mode != OFF && imu.available

    fun shiftFor(frameTimestampNanos: Long, frameWidth: Int, frameHeight: Int, isFrontCamera: Boolean): CameraShift {
        if (!active) return CameraShift.NONE
        val angles = imu.cameraAngles(frameTimestampNanos) ?: return CameraShift.NONE
        val halfAngle = if (isFrontCamera) halfAngleXDeg else halfAngleRearXDeg
        val perDegX = pixelsPerDegree(frameWidth / 2f, halfAngle)
        // The gyroscope's tilt is that of the screen: the front camera tips up
        // with it and the rear camera tips down. Pan is the same for both.
        val tiltSign = if (isFrontCamera) 1f else -1f
        val perDegY = if (mode == PAN_TILT) {
            tiltSign * pixelsPerDegree(frameHeight / 2f, verticalHalfAngleDeg(halfAngle, frameWidth, frameHeight))
        } else {
            0f
        }
        val sendPan = if (delayCompensation) angles.panNow else angles.panAtFrame
        val sendTilt = if (delayCompensation) angles.tiltNow else angles.tiltAtFrame
        return CameraShift(
            frameX = angles.panAtFrame * perDegX,
            frameY = angles.tiltAtFrame * perDegY,
            sendX = sendPan * perDegX,
            sendY = sendTilt * perDegY
        )
    }

    companion object {
        const val OFF = 0
        const val PAN = 1
        const val PAN_TILT = 2

        /** Measured on the rig on 2026-10-12: front camera, portrait, 480 x 640 analysis frame. */
        const val DEFAULT_HALF_ANGLE_X_DEG = 28.5f

        /** Small-angle scale from camera rotation to image shift. */
        fun pixelsPerDegree(halfSizePx: Float, halfAngleDeg: Float): Float = halfSizePx / halfAngleDeg

        /** Half-angle of the frame's other dimension, for square pixels. */
        fun verticalHalfAngleDeg(halfAngleXDeg: Float, frameWidth: Int, frameHeight: Int): Float {
            val t = tan(Math.toRadians(halfAngleXDeg.toDouble())) * frameHeight / frameWidth
            return Math.toDegrees(atan(t)).toFloat()
        }
    }
}
