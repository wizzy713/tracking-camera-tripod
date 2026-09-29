package com.example.tripodtracker

import android.graphics.Rect
import kotlin.math.hypot
import kotlin.math.max

/**
 * Assigns stable tracking IDs to ball detections across frames.
 *
 * MediaPipe's ObjectDetector (used for DetectionMode.BALL) is a per-frame
 * detector with no tracking IDs, but the lock-on gesture and the coast-on-loss
 * logic in processImageProxy both key off DetectedObjectInfo.trackingId. This
 * does greedy nearest-centre association: a detection whose centre is within
 * MATCH_RADIUS_FACTOR x (box size) of a live track inherits that track's ID,
 * otherwise it gets a fresh one. Tracks unmatched for more than
 * MAX_MISSED_FRAMES are dropped, so a ball that briefly drops out (motion blur
 * at the bottom of the pendulum swing) keeps its ID and the lock survives.
 *
 * Only ever touched from the single camera-analysis executor thread.
 */
class BallTracker {
    private data class Track(val id: Int, var box: Rect, var missed: Int = 0)

    private val tracks = mutableListOf<Track>()
    private var nextId = 1

    fun update(boxes: List<Rect>): List<Pair<Rect, Int>> {
        val unmatched = tracks.toMutableList()
        val result = boxes.map { box ->
            val best = unmatched.minByOrNull { centreDistance(it.box, box) }
            val radius = best?.let { MATCH_RADIUS_FACTOR * max(max(it.box.width(), it.box.height()), max(box.width(), box.height())) }
            if (best != null && radius != null && centreDistance(best.box, box) <= radius) {
                unmatched.remove(best)
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

    private fun centreDistance(a: Rect, b: Rect): Float =
        hypot(a.exactCenterX() - b.exactCenterX(), a.exactCenterY() - b.exactCenterY())

    private companion object {
        // A pendulum ball can move a few diameters between frames at 30 fps.
        const val MATCH_RADIUS_FACTOR = 2.5f
        // Matches MAX_COAST_FRAMES in MainActivity, so the ID outlives the lock.
        const val MAX_MISSED_FRAMES = 15
    }
}
