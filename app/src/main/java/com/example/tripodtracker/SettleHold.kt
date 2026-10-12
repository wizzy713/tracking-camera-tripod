package com.example.tripodtracker

import kotlin.math.abs

/**
 * Hold-still band with hysteresis, for one axis.
 *
 * The servos cannot turn slowly (they jump from stopped to a minimum speed at
 * the edge of their dead band) and the loop has ~0.4 s of delay, so with a
 * plain deadzone a subject who is standing still is hunted: detection jitter
 * pushes the error over the edge, the servo starts, overshoots during the
 * delay and has to come back.
 *
 * This gate separates "stop" from "start". While tracking, the error is passed
 * through until the predicted error comes within [apply]'s `enter` band; the
 * axis then holds (error 0, servo stopped). It stays held until the subject's
 * filtered position, without the prediction (whose velocity term is the noisy
 * part), has been outside the wider `exit` band for [EXIT_FRAMES] frames in a
 * row, i.e. until the subject has really moved and not just jittered.
 */
class SettleHold {
    var holding = false
        private set
    private var outsideFrames = 0

    fun reset() {
        holding = false
        outsideFrames = 0
    }

    /**
     * @param predictedError the error that would be sent (from the Kalman prediction)
     * @param measuredError the filtered position's error, with no prediction
     * @param enter stop once |predictedError| is within this
     * @param exit start again once |measuredError| is beyond this; the gate is
     *   off (the error is passed through) unless `exit > enter >= 0` and `exit > 0`
     * @param subjectMoving the subject is known to be moving (room-coordinate
     *   tracking only); the band then lets the error through, so that it does
     *   not stop the camera each time a moving subject crosses the centre
     * @return the error to send: 0 while holding
     */
    fun apply(
        predictedError: Float,
        measuredError: Float,
        enter: Float,
        exit: Float,
        subjectMoving: Boolean = false
    ): Float {
        if (exit <= 0f || enter < 0f || exit <= enter || subjectMoving) {
            reset()
            return predictedError
        }
        if (holding) {
            outsideFrames = if (abs(measuredError) > exit) outsideFrames + 1 else 0
            if (outsideFrames >= EXIT_FRAMES) reset()
        } else if (abs(predictedError) <= enter) {
            holding = true
            outsideFrames = 0
        }
        return if (holding) 0f else predictedError
    }

    companion object {
        const val EXIT_FRAMES = 2
    }
}
