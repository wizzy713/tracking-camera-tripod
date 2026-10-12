package com.example.tripodtracker

import kotlin.math.hypot
import kotlin.math.max

/**
 * Assigns stable tracking IDs to ball detections across frames.
 *
 * MediaPipe's ObjectDetector (used for DetectionMode.BALL) is a per-frame
 * detector with no tracking IDs, but the lock-on gesture and the coast-on-loss
 * logic in processImageProxy both key off DetectedObjectInfo.trackingId. This
 * does greedy nearest-centre association: a detection whose centre is within
 * MATCH_RADIUS_FACTOR x (box size) of a live track's PREDICTED centre inherits
 * that track's ID, otherwise it gets a fresh one. Tracks unmatched for more than
 * MAX_MISSED_FRAMES are dropped, so a ball that briefly drops out (motion blur
 * at the bottom of the pendulum swing) keeps its ID and the lock survives.
 *
 * The prediction is the track's last per-frame displacement carried forward
 * (constant velocity). Matching against the last-seen centre instead loses the
 * ID whenever the ball moves more than the radius between frames -- at the
 * bottom of a fast swing, or after a few dropped frames -- which dropped the
 * lock mid-test even though the ball was still being detected.
 *
 * Only ever touched from the single camera-analysis executor thread.
 */
class BallTracker {
    private class Track(val id: Int, var box: BallBox) {
        var vx = 0f      // centre displacement per frame, px
        var vy = 0f
        var missed = 0

        val predictedX: Float get() = box.centerX + vx * (missed + 1)
        val predictedY: Float get() = box.centerY + vy * (missed + 1)
    }

    private val tracks = mutableListOf<Track>()
    private var nextId = 1

    fun update(boxes: List<BallBox>): List<Pair<BallBox, Int>> {
        val unmatched = tracks.toMutableList()
        val result = boxes.map { box ->
            val best = unmatched.minByOrNull { distanceToPrediction(it, box) }
            val radius = best?.let { MATCH_RADIUS_FACTOR * max(max(it.box.width, it.box.height), max(box.width, box.height)) }
            if (best != null && radius != null && distanceToPrediction(best, box) <= radius) {
                unmatched.remove(best)
                val frames = best.missed + 1
                best.vx = (box.centerX - best.box.centerX) / frames
                best.vy = (box.centerY - best.box.centerY) / frames
                best.box = box
                best.missed = 0
                box to best.id
            } else {
                val track = Track(nextId++, box)
                tracks.add(track)
                box to track.id
            }
        }
        unmatched.forEach { it.missed++ }
        tracks.removeAll { it.missed > MAX_MISSED_FRAMES }
        return result
    }

    private fun distanceToPrediction(track: Track, box: BallBox): Float =
        hypot(track.predictedX - box.centerX, track.predictedY - box.centerY)

    private companion object {
        // A pendulum ball can move a few diameters between frames at 30 fps; the
        // radius now only has to cover the error in the constant-velocity guess.
        const val MATCH_RADIUS_FACTOR = 2.5f
        // Matches MAX_COAST_FRAMES in MainActivity, so the ID outlives the lock.
        const val MAX_MISSED_FRAMES = 15
    }
}
