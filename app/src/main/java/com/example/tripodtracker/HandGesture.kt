package com.example.tripodtracker

import kotlin.math.hypot

/** The hand poses the camera screen reacts to -- see [classifyHandGesture]. */
enum class HandGesture { NONE, OPEN_PALM, FIST, VICTORY }

/** One hand landmark in upright-frame pixels (not MediaPipe's normalized units,
 *  whose X and Y scales differ on a non-square frame). */
class HandPoint(val x: Float, val y: Float)

// MediaPipe hand model indices: (PIP joint, fingertip) for index..pinky.
private val FINGERS = listOf(6 to 8, 10 to 12, 14 to 16, 18 to 20)
private const val WRIST = 0
private const val MIDDLE_KNUCKLE = 9
// A straight finger's tip is about 1.4x as far from the wrist as its PIP joint;
// a curled one's tip is nearer the wrist than the joint. In between is neither.
private const val EXTENDED_RATIO = 1.2f

/**
 * Classifies a 21-landmark hand:
 *
 *  - OPEN_PALM: all four fingers pointing up (tip above its PIP joint), the
 *    lock-on rule the app has always used.
 *  - FIST: all four fingers curled.
 *  - VICTORY: index and middle straight and pointing up, ring and pinky curled.
 *
 * The thumb is not used: its landmarks are the least reliable of the hand.
 */
fun classifyHandGesture(hand: List<HandPoint>): HandGesture {
    if (hand.size < 21) return HandGesture.NONE
    val wrist = hand[WRIST]
    fun distance(i: Int) = hypot(hand[i].x - wrist.x, hand[i].y - wrist.y)

    // Every gesture needs a raised hand (knuckles above the wrist). A hand
    // hanging at the subject's side is upside down in the frame, where curled
    // fingers have their tips "above" the joints and would pass for an open palm.
    if (hand[MIDDLE_KNUCKLE].y >= wrist.y) return HandGesture.NONE

    val pointsUp = FINGERS.map { (pip, tip) -> hand[tip].y < hand[pip].y }
    val curled = FINGERS.map { (pip, tip) -> distance(tip) < distance(pip) }
    val straight = FINGERS.map { (pip, tip) -> distance(tip) > EXTENDED_RATIO * distance(pip) }

    return when {
        pointsUp.all { it } -> HandGesture.OPEN_PALM
        curled.all { it } -> HandGesture.FIST
        straight[0] && straight[1] && pointsUp[0] && pointsUp[1] && curled[2] && curled[3] -> HandGesture.VICTORY
        else -> HandGesture.NONE
    }
}

/**
 * Turns the per-check gesture stream into one-shot commands: a gesture fires
 * once it has been seen on [holdChecks] consecutive hand checks, and not again
 * until the hand has shown something else in between. This filters the odd
 * misclassified frame and stops a held fist from firing over and over.
 */
class GestureTrigger(private val holdChecks: Int = 2) {
    private var current = HandGesture.NONE
    private var count = 0
    private var fired = false

    /** Feed the gesture from each hand check; returns it on the check it fires, else null. */
    fun update(gesture: HandGesture): HandGesture? {
        if (gesture != current) {
            current = gesture
            count = 0
            fired = false
        }
        count++
        if (gesture != HandGesture.NONE && !fired && count >= holdChecks) {
            fired = true
            return gesture
        }
        return null
    }
}
