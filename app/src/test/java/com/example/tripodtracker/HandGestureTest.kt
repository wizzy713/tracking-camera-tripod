package com.example.tripodtracker

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class HandGestureTest {

    /**
     * Builds a synthetic upright hand (wrist at the bottom, y grows downwards).
     * Each finger is straight up, or curled so its tip folds back to the palm.
     * [flip] turns the whole hand upside down (fingers pointing at the floor).
     */
    private fun hand(index: Boolean, middle: Boolean, ring: Boolean, pinky: Boolean, flip: Boolean = false): List<HandPoint> {
        val pts = Array(21) { HandPoint(100f, 200f) } // wrist + unused thumb points
        val extended = listOf(index, middle, ring, pinky)
        for (f in 0 until 4) {
            val x = 80f + 15f * f
            val mcp = 5 + 4 * f
            pts[mcp] = HandPoint(x, 140f)
            pts[mcp + 1] = HandPoint(x, 110f)                              // PIP
            pts[mcp + 2] = HandPoint(x, if (extended[f]) 90f else 115f)    // DIP
            pts[mcp + 3] = HandPoint(x, if (extended[f]) 70f else 135f)    // tip
        }
        return pts.map { if (flip) HandPoint(it.x, 400f - it.y) else it }
    }

    @Test
    fun classifiesTheThreeGestures() {
        assertEquals(HandGesture.OPEN_PALM, classifyHandGesture(hand(true, true, true, true)))
        assertEquals(HandGesture.FIST, classifyHandGesture(hand(false, false, false, false)))
        assertEquals(HandGesture.VICTORY, classifyHandGesture(hand(true, true, false, false)))
    }

    @Test
    fun otherFingerCombinationsAreNotGestures() {
        assertEquals(HandGesture.NONE, classifyHandGesture(hand(true, false, false, false)))
        assertEquals(HandGesture.NONE, classifyHandGesture(hand(true, true, true, false)))
        assertEquals(HandGesture.NONE, classifyHandGesture(hand(false, true, true, true)))
    }

    @Test
    fun aHandHangingDownIsNotAGesture() {
        // Curled and upside down: fingertips are "above" the joints in the frame.
        assertEquals(HandGesture.NONE, classifyHandGesture(hand(false, false, false, false, flip = true)))
        assertEquals(HandGesture.NONE, classifyHandGesture(hand(true, true, true, true, flip = true)))
    }

    @Test
    fun incompleteLandmarksAreIgnored() {
        assertEquals(HandGesture.NONE, classifyHandGesture(hand(true, true, true, true).take(20)))
    }

    @Test
    fun triggerFiresOnceAfterTheGestureIsHeld() {
        val trigger = GestureTrigger(holdChecks = 2)
        assertNull(trigger.update(HandGesture.FIST))
        assertEquals(HandGesture.FIST, trigger.update(HandGesture.FIST))
        assertNull(trigger.update(HandGesture.FIST)) // still held: no repeat
        assertNull(trigger.update(HandGesture.NONE))
        assertNull(trigger.update(HandGesture.FIST))
        assertEquals(HandGesture.FIST, trigger.update(HandGesture.FIST))
    }

    @Test
    fun triggerIgnoresASingleMisclassifiedCheck() {
        val trigger = GestureTrigger(holdChecks = 2)
        assertNull(trigger.update(HandGesture.FIST))
        assertNull(trigger.update(HandGesture.OPEN_PALM))
        assertNull(trigger.update(HandGesture.FIST))
        assertNull(trigger.update(HandGesture.NONE))
    }
}
