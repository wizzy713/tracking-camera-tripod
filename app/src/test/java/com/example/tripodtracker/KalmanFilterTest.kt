package com.example.tripodtracker

import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.random.Random

class KalmanFilterTest {

    /**
     * Regression test for the covariance-update ordering bug: the correction step
     * once read p_p/p_pv AFTER they had already been overwritten by the position
     * update, which breaks P's symmetry and can drive its diagonal negative. A
     * mathematically correct Kalman update keeps P symmetric and positive on the
     * diagonal after every step.
     */
    @Test
    fun covarianceStaysSymmetricAndPositive() {
        val filter = KalmanFilter()
        val random = Random(42)
        var t = 0L

        repeat(500) {
            t += random.nextLong(5_000_000L, 40_000_000L) // 5-40ms between frames, in ns
            val measurement = random.nextFloat() * 640f
            filter.update(measurement, t)

            assertTrue("p_p should stay positive", filter.covariancePP > 0f)
            assertTrue("p_vv should stay positive", filter.covarianceVV > 0f)
            assertTrue(
                "p_pv (${filter.covariancePV}) should equal p_vp (${filter.covarianceVP})",
                abs(filter.covariancePV - filter.covarianceVP) < 1e-3f
            )
        }
    }

    @Test
    fun predictionReducesRmseVersusRawMeasurement() {
        val random = Random(7)
        val filter = KalmanFilter()

        val dtNanos = 33_333_333L // ~30 Hz
        val velocityPxPerSec = 120f
        val jitterStdDev = 8f
        val horizonSeconds = 0.1f
        val horizonNanos = (horizonSeconds * 1_000_000_000f).toLong()

        var t = 0L
        var truePosition = 0f
        var rawSquaredError = 0.0
        var predictedSquaredError = 0.0
        var samples = 0

        repeat(300) {
            t += dtNanos
            truePosition += velocityPxPerSec * (dtNanos / 1_000_000_000f)
            val measurement = truePosition + gaussian(random) * jitterStdDev

            filter.update(measurement, t)

            if (it > 20) { // let the filter converge past startup transients
                val trueFuturePosition = truePosition + velocityPxPerSec * horizonSeconds
                val predicted = filter.predictFuture(horizonSeconds)

                rawSquaredError += (measurement - trueFuturePosition).let { e -> e * e }
                predictedSquaredError += (predicted - trueFuturePosition).let { e -> e * e }
                samples++
            }
        }

        val rawRmse = sqrt(rawSquaredError / samples)
        val predictedRmse = sqrt(predictedSquaredError / samples)

        assertTrue(
            "predicted RMSE ($predictedRmse) should beat raw measurement RMSE ($rawRmse) " +
                "against ground truth $horizonSeconds s ahead",
            predictedRmse < rawRmse
        )
    }

    private fun gaussian(random: Random): Float {
        // Box-Muller transform; avoids a dependency on kotlin.random's Gaussian (JDK-only).
        val u1 = random.nextDouble().coerceAtLeast(1e-9)
        val u2 = random.nextDouble()
        return (sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2)).toFloat()
    }
}
