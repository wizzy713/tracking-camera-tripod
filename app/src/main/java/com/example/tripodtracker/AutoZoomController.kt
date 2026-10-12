package com.example.tripodtracker

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln

/**
 * Auto zoom: picks the camera zoom ratio that keeps the tracked subject at a
 * constant size in the frame.
 *
 * CameraX applies zoom to every use case, so the analysis frame the subject is
 * measured in is already zoomed. That closes the loop: the size seen at the
 * current ratio says directly how much more (or less) zoom is needed, and no
 * model of the lens or the subject's distance is required.
 *
 * The ratio moves geometrically (a fixed fraction of the remaining size error
 * per second, rate-limited) so a zoom looks the same speed at 1x as at 3x and
 * a noisy bounding box does not make the picture breathe.
 */
class AutoZoomController(
    /** Size error tolerated before zooming: within a factor of 1 + deadband of the target. */
    private val deadband: Float = 0.15f,
    /** Fraction of the remaining (log) size error closed per second. */
    private val gainPerSecond: Float = 1.5f,
    /** Fastest zoom change, as a (log) factor per second: 0.5 is about 1.65x/s. */
    private val maxLogRatePerSecond: Float = 0.5f,
    /** How fast to ease back out to the widest view once the subject is lost. */
    private val releaseLogRatePerSecond: Float = 0.25f,
    /** Zoom out if the subject's box reaches this far towards the frame edge. */
    private val edgeLimit: Float = 0.9f
) {
    /**
     * @param subjectFraction subject size as a fraction of the frame (the larger
     *   of its width and height fractions), or null when no subject is visible
     * @param edgeExtent how far the subject's farthest edge is from the frame
     *   centre, as a fraction of the half-frame (1 = touching the frame edge)
     * @param targetFraction the size to hold the subject at
     * @param currentRatio the camera's zoom ratio now
     * @param dtSeconds time since the last call
     * @return the zoom ratio to set, within [minRatio, maxRatio]
     */
    fun update(
        subjectFraction: Float?,
        edgeExtent: Float,
        targetFraction: Float,
        currentRatio: Float,
        minRatio: Float,
        maxRatio: Float,
        dtSeconds: Float
    ): Float {
        val dt = dtSeconds.coerceIn(0f, MAX_DT_SECONDS)
        val logStep = if (subjectFraction == null || subjectFraction <= 0f) {
            // Nothing to frame: widen so the subject is easier to find again.
            -releaseLogRatePerSecond * dt
        } else {
            // Never zoom the subject out of the picture: its farthest edge scales
            // with the zoom, so this is the most zoom that keeps it inside
            // edgeLimit. A subject already past that (off-centre, or simply too
            // big) is backed away from, deadband or not.
            val bySize = targetFraction / subjectFraction
            val byEdge = if (edgeExtent > 0f) edgeLimit / edgeExtent else Float.MAX_VALUE
            val logError = ln(minOf(bySize, byEdge))
            if (bySize <= byEdge && abs(logError) < ln(1f + deadband)) {
                0f
            } else {
                (gainPerSecond * logError * dt).coerceIn(-maxLogRatePerSecond * dt, maxLogRatePerSecond * dt)
            }
        }
        return (currentRatio * exp(logStep)).coerceIn(minRatio, maxOf(minRatio, maxRatio))
    }

    private companion object {
        // A stalled pipeline must not be spent as one big zoom jump.
        const val MAX_DT_SECONDS = 0.2f
    }
}
