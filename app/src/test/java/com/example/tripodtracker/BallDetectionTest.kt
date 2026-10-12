package com.example.tripodtracker

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos

class BallDetectionTest {

    private val w = 480
    private val h = 640
    private val background = 0xFF303030.toInt()
    private val yellow = 0xFFDCE628.toInt()   // optic yellow, hue ~63 deg

    private fun frame() = IntArray(w * h) { background }

    private fun IntArray.disc(cx: Int, cy: Int, r: Int) {
        for (y in cy - r..cy + r) for (x in cx - r..cx + r) {
            if (x in 0 until w && y in 0 until h && (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r) {
                this[y * w + x] = yellow
            }
        }
    }

    private fun IntArray.band(y0: Int, y1: Int) {
        for (y in y0 until y1) for (x in 0 until w) this[y * w + x] = background
    }

    @Test
    fun ballCutByStringIsOneDetection() {
        // A 12 px dark band (the string, ~4 grid cells) splits the ball into two
        // flat halves, each of which fails the roundness test on its own.
        val img = frame().apply { disc(240, 320, 40); band(314, 326) }
        val boxes = ColorBallDetector().detect(img, w, h)
        assertEquals(1, boxes.size)
        assertTrue(abs(boxes[0].centerX - 240) < 6 && abs(boxes[0].centerY - 320) < 6)
    }

    @Test
    fun smallYellowSpeckIsNotASecondBall() {
        val img = frame().apply { disc(240, 320, 40); disc(60, 80, 10) }
        assertEquals(1, ColorBallDetector().detect(img, w, h).size)
    }

    @Test
    fun twoRealBallsAreBothReported() {
        val img = frame().apply { disc(120, 320, 35); disc(360, 320, 35) }
        assertEquals(2, ColorBallDetector().detect(img, w, h).size)
    }

    @Test
    fun noYellowNoDetection() {
        assertTrue(ColorBallDetector().detect(frame(), w, h).isEmpty())
    }

    @Test
    fun swingingBallKeepsItsIdThroughTheFastBottomOfTheSwing() {
        // Pendulum released from rest: x = 240 - 220 cos(2 pi k / 16). At the
        // bottom the 30 px ball moves ~86 px/frame, beyond the 2.5 x 30 = 75 px
        // radius around where it was last seen (which used to drop the ID), but
        // close to the constant-velocity prediction.
        val tracker = BallTracker()
        val ids = (0 until 32).map { k ->
            val x = (240 - 220 * cos(2 * PI * k / 16)).toInt()
            tracker.update(listOf(BallBox(x - 15, 305, x + 15, 335))).single().second
        }
        assertEquals(1, ids.toSet().size)
    }

    @Test
    fun ballAndStaticDistractorKeepSeparateIds() {
        val tracker = BallTracker()
        val distractor = BallBox(400, 50, 440, 90)
        val ballIds = mutableSetOf<Int>()
        val distractorIds = mutableSetOf<Int>()
        for (k in 0 until 10) {
            val x = 60 + 25 * k
            val out = tracker.update(listOf(BallBox(x - 20, 400, x + 20, 440), distractor))
            ballIds += out[0].second
            distractorIds += out[1].second
        }
        assertEquals(1, ballIds.size)
        assertEquals(1, distractorIds.size)
        assertTrue(ballIds != distractorIds)
    }
}
