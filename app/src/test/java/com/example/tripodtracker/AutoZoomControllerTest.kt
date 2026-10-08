package com.example.tripodtracker

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class AutoZoomControllerTest {

    private val dt = 1f / 30f

    /** Runs the controller against a subject whose size scales with the zoom ratio. */
    private fun settle(sizeAt1x: Float, target: Float, startRatio: Float = 1f, maxRatio: Float = 3f): Float {
        val controller = AutoZoomController()
        var ratio = startRatio
        repeat(600) {
            val size = sizeAt1x * ratio
            ratio = controller.update(size, edgeExtent = size, targetFraction = target,
                currentRatio = ratio, minRatio = 1f, maxRatio = maxRatio, dtSeconds = dt)
        }
        return ratio
    }

    private fun assertWithinDeadband(size: Float, target: Float) {
        val factor = maxOf(size / target, target / size)
        assertTrue("size $size vs target $target", factor < 1.15f + 1e-3f)
    }

    @Test
    fun zoomsInUntilTheSubjectIsNearTheTargetSize() {
        val ratio = settle(sizeAt1x = 0.2f, target = 0.4f)
        assertWithinDeadband(size = 0.2f * ratio, target = 0.4f)
    }

    @Test
    fun zoomsBackOutWhenTheSubjectIsTooLarge() {
        val ratio = settle(sizeAt1x = 0.2f, target = 0.3f, startRatio = 3f)
        assertWithinDeadband(size = 0.2f * ratio, target = 0.3f)
    }

    @Test
    fun staysWithinTheZoomRange() {
        assertEquals(3f, settle(sizeAt1x = 0.02f, target = 0.5f), 1e-4f)
        assertEquals(1f, settle(sizeAt1x = 0.9f, target = 0.3f, startRatio = 2f), 1e-4f)
    }

    @Test
    fun holdsInsideTheDeadband() {
        val ratio = AutoZoomController().update(0.42f, edgeExtent = 0.42f, targetFraction = 0.4f,
            currentRatio = 1.7f, minRatio = 1f, maxRatio = 3f, dtSeconds = dt)
        assertEquals(1.7f, ratio, 0f)
    }

    @Test
    fun zoomsOutInsteadOfInWhenTheSubjectIsAtTheFrameEdge() {
        // Small subject, so size alone asks for more zoom, but its box touches the edge.
        val ratio = AutoZoomController().update(0.1f, edgeExtent = 1f, targetFraction = 0.4f,
            currentRatio = 2f, minRatio = 1f, maxRatio = 3f, dtSeconds = dt)
        assertTrue("expected zoom out, got $ratio", ratio < 2f)
    }

    @Test
    fun stopsZoomingInBeforeAnOffCentreSubjectReachesTheEdge() {
        // Subject 0.1 wide with its far edge 0.5 of the way out at 1x: the size
        // target alone would need 4x, the edge limit allows 1.8x.
        val controller = AutoZoomController()
        var ratio = 1f
        repeat(600) {
            ratio = controller.update(0.1f * ratio, edgeExtent = 0.5f * ratio, targetFraction = 0.4f,
                currentRatio = ratio, minRatio = 1f, maxRatio = 3f, dtSeconds = dt)
        }
        assertEquals(1.8f, ratio, 0.02f)
    }

    @Test
    fun easesBackToTheWidestViewWhenTheSubjectIsLost() {
        val controller = AutoZoomController()
        var ratio = 2.5f
        val first = controller.update(null, 0f, 0.4f, ratio, 1f, 3f, dt)
        assertTrue(first < ratio && first > 2.4f) // gradual, not a jump
        repeat(600) { ratio = controller.update(null, 0f, 0.4f, ratio, 1f, 3f, dt) }
        assertEquals(1f, ratio, 1e-4f)
    }

    @Test
    fun aLongGapBetweenFramesIsNotSpentAsOneJump() {
        val ratio = AutoZoomController().update(0.05f, edgeExtent = 0.05f, targetFraction = 0.5f,
            currentRatio = 1f, minRatio = 1f, maxRatio = 3f, dtSeconds = 5f)
        assertTrue("got $ratio", ratio < 1.15f)
    }
}
