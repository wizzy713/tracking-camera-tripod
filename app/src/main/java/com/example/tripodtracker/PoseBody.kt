package com.example.tripodtracker

/** One body landmark in upright-frame pixels, with MediaPipe's visibility score (0-1). */
class PosePoint(val x: Float, val y: Float, val visibility: Float)

/**
 * A person found by the pose model: the box around the visible body, the point
 * to keep centred, and the landmarks themselves (null where not visible) for
 * the skeleton overlay.
 */
class PoseBody(val box: BallBox, val aimX: Float, val aimY: Float, val skeleton: List<PosePoint?>)

// MediaPipe pose model indices.
private const val LEFT_SHOULDER = 11
private const val RIGHT_SHOULDER = 12
private const val LEFT_HIP = 23
private const val RIGHT_HIP = 24

/** Limb segments drawn by the skeleton overlay (indices into the 33 landmarks). */
val POSE_CONNECTIONS = listOf(
    11 to 12, 11 to 23, 12 to 24, 23 to 24, // torso
    11 to 13, 13 to 15, 12 to 14, 14 to 16, // arms
    23 to 25, 25 to 27, 24 to 26, 26 to 28  // legs
)

private const val MIN_VISIBILITY = 0.5f
// A pose with fewer visible landmarks than this, or with no visible torso, is
// treated as a false detection rather than a person.
private const val MIN_VISIBLE_LANDMARKS = 6
// Landmarks are joints, so the body extends past them (top of the head, hands,
// feet): the box is grown by this fraction of its height on every side.
private const val BOX_PADDING_FRACTION = 0.08f
// Aim point: this far from the shoulder line towards the hip line, i.e. the
// chest. Aiming at the middle of the body instead pushes the head out of the top
// of the frame whenever the subject is close.
private const val AIM_SHOULDER_TO_HIP_FRACTION = 0.25f

/**
 * Converts one pose (33 landmarks) into a tracked body, or null if too little of
 * it is visible to be a person. Unlike a detector's bounding box, the aim point
 * comes from the torso landmarks, so it does not move when the subject waves an
 * arm or takes a step.
 */
fun poseToBody(landmarks: List<PosePoint>, frameWidth: Int, frameHeight: Int): PoseBody? {
    if (landmarks.size < 33) return null
    val skeleton = landmarks.map { p ->
        p.takeIf {
            it.visibility >= MIN_VISIBILITY && it.x >= 0f && it.x <= frameWidth && it.y >= 0f && it.y <= frameHeight
        }
    }
    val visible = skeleton.filterNotNull()
    val shoulders = listOfNotNull(skeleton[LEFT_SHOULDER], skeleton[RIGHT_SHOULDER])
    val hips = listOfNotNull(skeleton[LEFT_HIP], skeleton[RIGHT_HIP])
    if (visible.size < MIN_VISIBLE_LANDMARKS || (shoulders.size < 2 && hips.size < 2)) return null

    val pad = BOX_PADDING_FRACTION * (visible.maxOf { it.y } - visible.minOf { it.y })
    val box = BallBox(
        left = (visible.minOf { it.x } - pad).coerceAtLeast(0f).toInt(),
        top = (visible.minOf { it.y } - pad).coerceAtLeast(0f).toInt(),
        right = (visible.maxOf { it.x } + pad).coerceAtMost(frameWidth.toFloat()).toInt(),
        bottom = (visible.maxOf { it.y } + pad).coerceAtMost(frameHeight.toFloat()).toInt()
    )
    if (box.width <= 0 || box.height <= 0) return null

    val torso = shoulders + hips
    val aimX = torso.map { it.x }.average().toFloat()
    val aimY = if (shoulders.size == 2) {
        val shoulderY = (shoulders[0].y + shoulders[1].y) / 2f
        // With the hips out of view (subject close, or cut off by the frame), a
        // shoulder width below the shoulder line stands in for the torso length.
        val torsoLength = if (hips.size == 2) {
            (hips[0].y + hips[1].y) / 2f - shoulderY
        } else {
            2f * kotlin.math.abs(shoulders[0].x - shoulders[1].x)
        }
        shoulderY + AIM_SHOULDER_TO_HIP_FRACTION * torsoLength
    } else {
        box.top + 0.25f * box.height
    }
    return PoseBody(box, aimX, aimY.coerceIn(box.top.toFloat(), box.bottom.toFloat()), skeleton)
}
