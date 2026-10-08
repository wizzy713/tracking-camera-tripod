package com.example.tripodtracker

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PoseBodyTest {

    private val w = 480
    private val h = 640

    /** A standing person: shoulders at y=200, hips at y=350, feet at y=600. */
    private fun standing(): MutableList<PosePoint> {
        val pts = MutableList(33) { PosePoint(0f, 0f, 0f) } // not visible unless set below
        fun set(i: Int, x: Float, y: Float) { pts[i] = PosePoint(x, y, 0.9f) }
        set(0, 240f, 150f)                          // nose
        set(11, 280f, 200f); set(12, 200f, 200f)    // shoulders
        set(13, 300f, 280f); set(14, 180f, 280f)    // elbows
        set(15, 310f, 350f); set(16, 170f, 350f)    // wrists
        set(23, 265f, 350f); set(24, 215f, 350f)    // hips
        set(25, 265f, 470f); set(26, 215f, 470f)    // knees
        set(27, 265f, 600f); set(28, 215f, 600f)    // ankles
        return pts
    }

    @Test
    fun boxCoversTheVisibleBodyWithPadding() {
        val body = poseToBody(standing(), w, h)!!
        assertTrue(body.box.left < 170 && body.box.right > 310)
        assertTrue(body.box.top < 150 && body.box.bottom > 600)
        assertTrue(body.box.left >= 0 && body.box.bottom <= h)
    }

    @Test
    fun aimsAtTheChest() {
        val body = poseToBody(standing(), w, h)!!
        assertEquals(240f, body.aimX, 0.5f)
        assertEquals(200f + 0.25f * 150f, body.aimY, 0.5f)
    }

    @Test
    fun aimDoesNotMoveWhenAnArmIsRaised() {
        val before = poseToBody(standing(), w, h)!!
        val waving = standing().also { it[15] = PosePoint(400f, 60f, 0.9f) }
        val after = poseToBody(waving, w, h)!!
        assertTrue("the box grows with the arm", after.box.right > before.box.right)
        assertEquals(before.aimX, after.aimX, 0f)
        assertEquals(before.aimY, after.aimY, 0f)
    }

    @Test
    fun upperBodyOnlyStillCounts() {
        val pts = standing()
        for (i in 23..28) pts[i] = PosePoint(pts[i].x, pts[i].y, 0.1f) // hips and legs out of view
        val body = poseToBody(pts, w, h)
        assertNotNull(body)
        assertEquals(200f + 0.25f * 2f * 80f, body!!.aimY, 0.5f)
        assertNull(body.skeleton[23])
    }

    @Test
    fun rejectsPosesWithNoVisibleTorsoOrTooFewLandmarks() {
        val noTorso = standing().also { for (i in listOf(11, 23)) it[i] = PosePoint(0f, 0f, 0f) }
        assertNull(poseToBody(noTorso, w, h))
        val sparse = MutableList(33) { PosePoint(0f, 0f, 0f) }
        sparse[11] = PosePoint(280f, 200f, 0.9f); sparse[12] = PosePoint(200f, 200f, 0.9f)
        assertNull(poseToBody(sparse, w, h))
    }

    @Test
    fun landmarksOutsideTheFrameAreNotUsed() {
        val pts = standing().also { it[27] = PosePoint(265f, 900f, 0.9f) }
        val body = poseToBody(pts, w, h)!!
        assertTrue(body.box.bottom <= h)
        assertNull(body.skeleton[27])
    }
}
