package com.example.tripodtracker

import android.graphics.Bitmap
import kotlin.math.max
import kotlin.math.min

/**
 * A ball bounding box in pixels (right/bottom exclusive, like android.graphics.Rect).
 * A plain class rather than Rect so the detector and BallTracker run in plain JVM
 * unit tests, where Android's Rect is an unimplemented stub.
 */
data class BallBox(val left: Int, val top: Int, val right: Int, val bottom: Int) {
    val width: Int get() = right - left
    val height: Int get() = bottom - top
    val centerX: Float get() = (left + right) / 2f
    val centerY: Float get() = (top + bottom) / 2f
}

/**
 * Finds a fluorescent-yellow tennis ball by colour, for DetectionMode.BALL.
 *
 * The COCO "sports ball" model recognises a ball by its overall appearance, which
 * a net wrapped around it (and the string it hangs from) breaks up badly. The
 * ball's optic-yellow colour survives the net, though: the strands only cover
 * thin lines of it. So this samples the frame on a coarse grid, marks cells whose
 * HSV colour is in the tennis-ball range, dilates the mask to bridge the gaps the
 * net strands leave, and keeps connected blobs that are roughly round and filled.
 *
 * Two clean-ups keep it to one box per ball. A string or net seam wider than the
 * dilation splits the ball into fragments (each too flat to pass the roundness
 * test on its own), so blobs lying within MERGE_GAP_FACTOR of each other are
 * merged before the shape filters run. And yellow specks much smaller than the
 * biggest blob (reflections, a shirt print) are dropped as distractors.
 *
 * Boxes are returned in the input bitmap's pixel frame, largest first. Not
 * thread-safe (reuses buffers); only ever touched from the camera-analysis
 * executor thread, like BallTracker.
 */
class ColorBallDetector(
    // Optic yellow is ~70 deg; the range is widened for shadow (greener) and
    // warm indoor light (more yellow). Skin (~20-30 deg) stays outside it.
    private val hueMin: Float = 45f,
    private val hueMax: Float = 85f,
    private val satMin: Float = 0.40f,
    private val valMin: Float = 0.30f,
) {
    private var pixels = IntArray(0)
    private var mask = BooleanArray(0)
    private var closed = BooleanArray(0)
    private var visited = BooleanArray(0)
    private var stack = IntArray(0)

    fun detect(bitmap: Bitmap): List<BallBox> {
        val w = bitmap.width
        val h = bitmap.height
        if (pixels.size < w * h) pixels = IntArray(w * h)
        bitmap.getPixels(pixels, 0, w, 0, 0, w, h)
        return detect(pixels, w, h)
    }

    /** Core of [detect] on a row-major ARGB buffer, so it can be unit-tested. */
    internal fun detect(argb: IntArray, w: Int, h: Int): List<BallBox> {
        // ~160 cells across the short side: plenty for a ball that fills a few
        // percent of the frame, and ~20k colour tests per frame instead of ~300k.
        val step = max(1, min(w, h) / GRID_CELLS_SHORT_SIDE)
        val gw = w / step
        val gh = h / step
        val n = gw * gh
        if (n == 0) return emptyList()

        if (mask.size < n) {
            mask = BooleanArray(n); closed = BooleanArray(n)
            visited = BooleanArray(n); stack = IntArray(n)
        }

        for (gy in 0 until gh) {
            val row = (gy * step + step / 2) * w
            for (gx in 0 until gw) {
                mask[gy * gw + gx] = isBallColour(argb[row + gx * step + step / 2])
            }
        }

        // Dilate so net strands crossing the ball don't split it into fragments.
        for (gy in 0 until gh) for (gx in 0 until gw) {
            var hit = false
            val y0 = max(0, gy - DILATE); val y1 = min(gh - 1, gy + DILATE)
            val x0 = max(0, gx - DILATE); val x1 = min(gw - 1, gx + DILATE)
            var y = y0
            while (!hit && y <= y1) {
                var x = x0
                while (!hit && x <= x1) { hit = mask[y * gw + x]; x++ }
                y++
            }
            closed[gy * gw + gx] = hit
        }

        java.util.Arrays.fill(visited, 0, n, false)
        val blobs = mutableListOf<Blob>()
        for (start in 0 until n) {
            if (!closed[start] || visited[start]) continue
            // Iterative 4-connected flood fill over the closed mask; count only
            // the original (undilated) hits so fill ratio reflects real colour.
            var sp = 0
            stack[sp++] = start
            visited[start] = true
            var minX = gw; var minY = gh; var maxX = -1; var maxY = -1; var hits = 0
            while (sp > 0) {
                val i = stack[--sp]
                val x = i % gw; val y = i / gw
                if (mask[i]) hits++
                if (x < minX) minX = x; if (x > maxX) maxX = x
                if (y < minY) minY = y; if (y > maxY) maxY = y
                if (x > 0) sp = push(i - 1, sp)
                if (x < gw - 1) sp = push(i + 1, sp)
                if (y > 0) sp = push(i - gw, sp)
                if (y < gh - 1) sp = push(i + gw, sp)
            }
            // Undo the dilation's growth of the bounding box.
            minX = min(minX + DILATE, maxX); maxX = max(maxX - DILATE, minX)
            minY = min(minY + DILATE, maxY); maxY = max(maxY - DILATE, minY)
            if (hits > 0) blobs += Blob(minX, minY, maxX, maxY, hits)
        }

        mergeFragments(blobs)

        val balls = blobs.filter { b ->
            val aspect = b.w.toFloat() / b.h
            // A clean disc fills ~78% of its box; the net and blur lower that.
            b.hits >= MIN_CELLS &&
                aspect >= MIN_ASPECT && aspect <= 1f / MIN_ASPECT &&
                b.hits.toFloat() / (b.w * b.h) >= MIN_FILL
        }
        val largest = balls.maxOfOrNull { it.hits } ?: return emptyList()
        return balls
            .filter { it.hits >= MIN_RELATIVE_SIZE * largest }
            .sortedByDescending { it.hits }
            .take(MAX_RESULTS)
            .map { BallBox(it.minX * step, it.minY * step, (it.maxX + 1) * step, (it.maxY + 1) * step) }
    }

    /** Blob bounds in grid cells (inclusive) and its count of ball-coloured cells. */
    private class Blob(var minX: Int, var minY: Int, var maxX: Int, var maxY: Int, var hits: Int) {
        val w get() = maxX - minX + 1
        val h get() = maxY - minY + 1
    }

    /**
     * Merges blobs whose boxes are closer than MERGE_GAP_FACTOR x the larger
     * one's size -- the halves of a ball cut by its string or a net seam. Repeats
     * until nothing merges, so a ball cut into three pieces still ends up whole.
     */
    private fun mergeFragments(blobs: MutableList<Blob>) {
        var merged = true
        while (merged) {
            merged = false
            outer@ for (i in blobs.indices) for (j in i + 1 until blobs.size) {
                val a = blobs[i]; val b = blobs[j]
                val gapX = max(0, max(a.minX, b.minX) - min(a.maxX, b.maxX) - 1)
                val gapY = max(0, max(a.minY, b.minY) - min(a.maxY, b.maxY) - 1)
                val size = max(max(a.w, a.h), max(b.w, b.h))
                if (max(gapX, gapY) <= MERGE_GAP_FACTOR * size) {
                    a.minX = min(a.minX, b.minX); a.minY = min(a.minY, b.minY)
                    a.maxX = max(a.maxX, b.maxX); a.maxY = max(a.maxY, b.maxY)
                    a.hits += b.hits
                    blobs.removeAt(j)
                    merged = true
                    break@outer
                }
            }
        }
    }

    private fun push(i: Int, sp: Int): Int {
        if (!closed[i] || visited[i]) return sp
        visited[i] = true
        stack[sp] = i
        return sp + 1
    }

    private fun isBallColour(argb: Int): Boolean {
        val r = (argb shr 16) and 0xFF
        val g = (argb shr 8) and 0xFF
        val b = argb and 0xFF
        val mx = max(r, max(g, b))
        val mn = min(r, min(g, b))
        if (mx < valMin * 255f) return false
        val delta = mx - mn
        if (delta < satMin * mx) return false
        // Yellow-green hues have green or red as the max channel and blue as min.
        if (mn != b) return false
        val hue = if (mx == g) 60f * ((b - r).toFloat() / delta + 2f) else 60f * ((g - b).toFloat() / delta)
        return hue in hueMin..hueMax
    }

    private companion object {
        const val GRID_CELLS_SHORT_SIDE = 160
        // Cells of dilation; bridges net strands up to ~2 cells (~2% of frame) wide.
        const val DILATE = 1
        const val MIN_CELLS = 12
        const val MIN_ASPECT = 0.5f
        const val MIN_FILL = 0.35f
        const val MAX_RESULTS = 5
        // Fragments separated by up to 25% of the ball's size are one ball (a
        // string or net seam is a few percent of it); a separate object would
        // need to sit almost touching the ball to be merged with it.
        const val MERGE_GAP_FACTOR = 0.25f
        // Blobs with under a quarter of the biggest blob's yellow cells are specks
        // and reflections, not a second ball.
        const val MIN_RELATIVE_SIZE = 0.25f
    }
}
