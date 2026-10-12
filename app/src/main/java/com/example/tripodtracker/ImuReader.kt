package com.example.tripodtracker

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import kotlin.math.asin
import kotlin.math.sqrt

/**
 * How the camera itself moved during one frame, from the phone's gyroscope.
 *
 * @param panDps rotation rate about the vertical, in deg/s; positive is
 *   anticlockwise seen from above.
 * @param tiltDps rotation rate about the horizontal axis across the phone, in
 *   deg/s; positive is the screen (front camera) tipping up. NaN when the phone
 *   lies flat, where that axis is undefined.
 * @param tiltDeg angle of the screen normal above the horizontal, from gravity.
 * @param ageMs frame timestamp minus the timestamp of the newest gyroscope
 *   sample used. Small and steady if the camera and the sensors share a clock.
 */
data class ImuSample(val panDps: Float, val tiltDps: Float, val tiltDeg: Float, val ageMs: Float) {
    companion object {
        val NONE = ImuSample(Float.NaN, Float.NaN, Float.NaN, Float.NaN)
    }
}

object ImuMath {
    private const val RAD_TO_DEG = 57.29578f

    /**
     * Splits a gyroscope reading into pan and tilt rates.
     *
     * @param gx/@param gy/@param gz angular velocity in rad/s, device axes
     * @param ux/@param uy/@param uz the "up" direction in device axes (what the
     *   gravity sensor reports), any length
     * @return (panDps, tiltDps), or null if there is no up direction yet
     */
    fun panTiltRates(gx: Float, gy: Float, gz: Float, ux: Float, uy: Float, uz: Float): Pair<Float, Float>? {
        val norm = sqrt(ux * ux + uy * uy + uz * uz)
        if (norm < 1e-3f) return null
        val x = ux / norm
        val y = uy / norm
        val z = uz / norm
        val pan = (gx * x + gy * y + gz * z) * RAD_TO_DEG
        // Horizontal axis across the phone: (screen normal) x (up) = (-y, x, 0).
        val h = sqrt(x * x + y * y)
        val tilt = if (h < 0.1f) Float.NaN else (-gx * y + gy * x) / h * RAD_TO_DEG
        return pan to tilt
    }

    /** Angle of the screen normal above the horizontal, in degrees. */
    fun tiltDeg(ux: Float, uy: Float, uz: Float): Float {
        val norm = sqrt(ux * ux + uy * uy + uz * uz)
        if (norm < 1e-3f) return Float.NaN
        return asin((uz / norm).coerceIn(-1f, 1f)) * RAD_TO_DEG
    }
}

/**
 * The most recent pan and tilt rates, with their sensor timestamps, and the
 * angles they add up to. The angles start at zero with the first sample, so
 * only their changes mean anything. Not thread-safe.
 */
class GyroBuffer(private val capacity: Int = 512) {
    private val times = LongArray(capacity)
    private val pan = FloatArray(capacity)
    private val tilt = FloatArray(capacity)
    private val panAngle = DoubleArray(capacity)
    private val tiltAngle = DoubleArray(capacity)
    private var next = 0
    private var size = 0

    fun add(timestampNanos: Long, panDps: Float, tiltDps: Float) {
        var panSum = 0.0
        var tiltSum = 0.0
        if (size > 0) {
            val last = (next - 1 + capacity) % capacity
            // A long gap means samples were missed; do not invent a rotation for it.
            val dt = ((timestampNanos - times[last]) / 1e9).coerceIn(0.0, MAX_STEP_SECONDS)
            panSum = panAngle[last] + panDps * dt
            tiltSum = tiltAngle[last] + (if (tiltDps.isNaN()) 0f else tiltDps) * dt
        }
        times[next] = timestampNanos
        pan[next] = panDps
        tilt[next] = tiltDps
        panAngle[next] = panSum
        tiltAngle[next] = tiltSum
        next = (next + 1) % capacity
        if (size < capacity) size++
    }

    /**
     * (pan, tilt) angle in degrees at a time, interpolated between samples; the
     * oldest or newest sample's angle outside the buffered span. Null if empty.
     */
    fun angleAt(timestampNanos: Long): Pair<Float, Float>? {
        if (size == 0) return null
        var later = -1
        for (k in 0 until size) {
            val i = (next - 1 - k + 2 * capacity) % capacity
            if (times[i] <= timestampNanos) {
                if (later < 0 || times[later] == times[i]) return panAngle[i].toFloat() to tiltAngle[i].toFloat()
                val f = (timestampNanos - times[i]).toDouble() / (times[later] - times[i])
                return (panAngle[i] + f * (panAngle[later] - panAngle[i])).toFloat() to
                    (tiltAngle[i] + f * (tiltAngle[later] - tiltAngle[i])).toFloat()
            }
            later = i
        }
        return panAngle[later].toFloat() to tiltAngle[later].toFloat()
    }

    /** (pan, tilt) angle in degrees at the newest sample, or null if empty. */
    fun latestAngle(): Pair<Float, Float>? {
        if (size == 0) return null
        val i = (next - 1 + capacity) % capacity
        return panAngle[i].toFloat() to tiltAngle[i].toFloat()
    }

    /**
     * Mean rates over the samples with a timestamp in (fromNanos, toNanos]. If
     * there are none, the newest sample at or before toNanos is used, and
     * failing that the newest sample of all.
     *
     * @return (panDps, tiltDps, timestamp of the newest sample used), or null if empty
     */
    fun mean(fromNanos: Long, toNanos: Long): Triple<Float, Float, Long>? {
        if (size == 0) return null
        var panSum = 0f
        var tiltSum = 0f
        var count = 0
        var newestInWindow = Long.MIN_VALUE
        var before = -1
        var newest = -1
        for (k in 0 until size) {
            val i = (next - 1 - k + 2 * capacity) % capacity
            val t = times[i]
            if (newest < 0) newest = i
            if (t <= toNanos) {
                if (before < 0) before = i
                if (t <= fromNanos) break
                panSum += pan[i]
                tiltSum += tilt[i]
                count++
                if (t > newestInWindow) newestInWindow = t
            }
        }
        if (count > 0) return Triple(panSum / count, tiltSum / count, newestInWindow)
        val i = if (before >= 0) before else newest
        return Triple(pan[i], tilt[i], times[i])
    }

    private companion object {
        const val MAX_STEP_SECONDS = 0.05
    }
}

/** The camera's pan and tilt angles, in degrees, when a frame was captured and at the newest gyroscope sample. */
data class CameraAngles(val panAtFrame: Float, val tiltAtFrame: Float, val panNow: Float, val tiltNow: Float)

/**
 * Reads the gyroscope and the gravity direction, and reports how the camera
 * turned during each frame (logged, see LogManager) and where it points (used
 * by [RoomFrame] when room-coordinate tracking is switched on).
 */
class ImuReader(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val gyroscope: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val gravity: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
        ?: sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private var thread: HandlerThread? = null

    private val lock = Any()
    private val buffer = GyroBuffer()
    private var upX = 0f
    private var upY = 0f
    private var upZ = 0f
    private var lastFrameNanos = 0L

    val available: Boolean get() = gyroscope != null && gravity != null

    fun start() {
        if (!available || thread != null) return
        val t = HandlerThread("ImuReader").also { it.start() }
        thread = t
        val handler = Handler(t.looper)
        sensorManager.registerListener(this, gyroscope, GYRO_PERIOD_US, handler)
        sensorManager.registerListener(this, gravity, GRAVITY_PERIOD_US, handler)
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        thread?.quitSafely()
        thread = null
    }

    override fun onSensorChanged(event: SensorEvent) {
        synchronized(lock) {
            when (event.sensor.type) {
                Sensor.TYPE_GRAVITY -> {
                    upX = event.values[0]; upY = event.values[1]; upZ = event.values[2]
                }
                Sensor.TYPE_ACCELEROMETER -> {
                    // No gravity sensor: smooth the accelerometer instead.
                    upX += ACCEL_SMOOTHING * (event.values[0] - upX)
                    upY += ACCEL_SMOOTHING * (event.values[1] - upY)
                    upZ += ACCEL_SMOOTHING * (event.values[2] - upZ)
                }
                Sensor.TYPE_GYROSCOPE -> {
                    val rates = ImuMath.panTiltRates(
                        event.values[0], event.values[1], event.values[2], upX, upY, upZ
                    ) ?: return
                    buffer.add(event.timestamp, rates.first, rates.second)
                }
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    /** Where the camera pointed when this frame was captured, and where it points now. Null before the first sample. */
    fun cameraAngles(frameTimestampNanos: Long): CameraAngles? = synchronized(lock) {
        val atFrame = buffer.angleAt(frameTimestampNanos) ?: return null
        val now = buffer.latestAngle() ?: return null
        CameraAngles(atFrame.first, atFrame.second, now.first, now.second)
    }

    /** The camera's motion over the interval that ended at this frame. */
    fun sampleFor(frameTimestampNanos: Long): ImuSample = synchronized(lock) {
        val previous = lastFrameNanos
        lastFrameNanos = frameTimestampNanos
        val gap = frameTimestampNanos - previous
        val from = if (previous == 0L || gap <= 0L || gap > MAX_FRAME_GAP_NANOS) {
            frameTimestampNanos - DEFAULT_FRAME_NANOS
        } else {
            previous
        }
        val mean = buffer.mean(from, frameTimestampNanos) ?: return ImuSample.NONE
        ImuSample(
            panDps = mean.first,
            tiltDps = mean.second,
            tiltDeg = ImuMath.tiltDeg(upX, upY, upZ),
            ageMs = (frameTimestampNanos - mean.third) / 1e6f
        )
    }

    private companion object {
        const val GYRO_PERIOD_US = 5_000          // 200 Hz
        const val GRAVITY_PERIOD_US = 20_000      // 50 Hz
        const val ACCEL_SMOOTHING = 0.1f
        const val DEFAULT_FRAME_NANOS = 33_333_333L
        const val MAX_FRAME_GAP_NANOS = 200_000_000L
    }
}
