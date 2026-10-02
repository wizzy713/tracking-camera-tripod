package com.example.tripodtracker

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import java.util.Locale

// Mirrors the .ino's shipped defaults (camx_tripod.ino) -- shown until the first
// CFG reply arrives from the tripod (see MainActivity.tripodConfig).
private val UNSYNCED_DEFAULT_CONFIG = TripodConfig(
    kp = 110f, ki = 50f, kd = 5f, maxSpeedOffsetUs = 220f, deadzone = 0.03f
)

/**
 * Live tuning surface for the PID controller (firmware, over UDP) and the
 * Kalman filter + prediction horizon (app-local), plus labeled CSV logging for
 * a physical test run. See TESTING.md for suggested experiments (step response,
 * pendulum, occlusion/coast, etc.) that this screen is meant to support.
 */
@Composable
fun ExperimentScreen(
    tripodConfig: TripodConfig?,
    onApplyPid: (TripodConfig) -> Unit,
    onSyncFromTripod: () -> Unit,
    kalmanFilterX: KalmanFilter,
    kalmanFilterY: KalmanFilter,
    predictionHorizonSeconds: Float,
    onPredictionHorizonChange: (Float) -> Unit,
    detectionMode: DetectionMode,
    onDetectionModeChange: (DetectionMode) -> Unit,
    isLogging: Boolean,
    onToggleLogging: (Boolean) -> Unit,
    testName: String,
    onTestNameChange: (String) -> Unit,
    testNotes: String,
    onTestNotesChange: (String) -> Unit,
    onBack: () -> Unit
) {
    BackHandler(onBack = onBack)

    var kp by remember { mutableFloatStateOf(tripodConfig?.kp ?: UNSYNCED_DEFAULT_CONFIG.kp) }
    var ki by remember { mutableFloatStateOf(tripodConfig?.ki ?: UNSYNCED_DEFAULT_CONFIG.ki) }
    var kd by remember { mutableFloatStateOf(tripodConfig?.kd ?: UNSYNCED_DEFAULT_CONFIG.kd) }
    var maxSpeedOffsetUs by remember { mutableFloatStateOf(tripodConfig?.maxSpeedOffsetUs ?: UNSYNCED_DEFAULT_CONFIG.maxSpeedOffsetUs) }
    var deadzone by remember { mutableFloatStateOf(tripodConfig?.deadzone ?: UNSYNCED_DEFAULT_CONFIG.deadzone) }

    // Any CFG reply (an "Apply" echo or a "Sync" query response) re-seeds the
    // sliders, so the UI always reflects what the tripod actually has -- e.g.
    // after a Serial-side tune, or after reconnecting to an already-running rig.
    LaunchedEffect(tripodConfig) {
        tripodConfig?.let {
            kp = it.kp
            ki = it.ki
            kd = it.kd
            maxSpeedOffsetUs = it.maxSpeedOffsetUs
            deadzone = it.deadzone
        }
    }

    var measurementNoise by remember { mutableFloatStateOf(kalmanFilterX.measurementNoise) }
    var accelerationNoise by remember { mutableFloatStateOf(kalmanFilterX.accelerationNoise) }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .verticalScroll(rememberScrollState())
            .padding(16.dp)
    ) {
        Text("Experiment", style = MaterialTheme.typography.headlineMedium, color = MaterialTheme.colorScheme.onBackground)
        Spacer(modifier = Modifier.height(16.dp))

        SectionCard(title = "Tripod PID Gains") {
            if (tripodConfig == null) {
                Text(
                    "Not yet synced -- showing firmware defaults. Tap \"Sync from Tripod\" once connected.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
                Spacer(modifier = Modifier.height(8.dp))
            }

            // KP/KI/KD maxima match the firmware's KP_LIMIT/KI_LIMIT/KD_LIMIT power-safety caps.
            GainSlider("KP", kp, 0f..200f) { kp = it }
            GainSlider("KI", ki, 0f..100f) { ki = it }
            GainSlider("KD", kd, 0f..20f) { kd = it }
            GainSlider("MAX_SPEED_OFFSET_US", maxSpeedOffsetUs, 10f..400f, decimals = 0) { maxSpeedOffsetUs = it }
            GainSlider("DEADZONE", deadzone, 0f..0.2f, decimals = 3) { deadzone = it }

            Spacer(modifier = Modifier.height(8.dp))
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                Button(onClick = onSyncFromTripod) { Text("Sync from Tripod") }
                Button(onClick = {
                    onApplyPid(TripodConfig(kp, ki, kd, maxSpeedOffsetUs, deadzone))
                }) { Text("Apply to Tripod") }
            }
        }

        Spacer(modifier = Modifier.height(16.dp))

        SectionCard(title = "Kalman Filter (local, both axes)") {
            GainSlider("Measurement noise R", measurementNoise, 1f..500f, decimals = 0) {
                measurementNoise = it
                kalmanFilterX.measurementNoise = it
                kalmanFilterY.measurementNoise = it
            }
            GainSlider("Acceleration noise σₐ", accelerationNoise, 1f..500f, decimals = 0) {
                accelerationNoise = it
                kalmanFilterX.accelerationNoise = it
                kalmanFilterY.accelerationNoise = it
            }
            GainSlider("Prediction horizon (s)", predictionHorizonSeconds, 0f..0.5f, decimals = 3) {
                onPredictionHorizonChange(it)
            }
        }

        Spacer(modifier = Modifier.height(16.dp))

        SectionCard(title = "Detection Mode") {
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
                FilterChip(
                    selected = detectionMode == DetectionMode.FACE,
                    onClick = { onDetectionModeChange(DetectionMode.FACE) },
                    label = { Text("Face") }
                )
                FilterChip(
                    selected = detectionMode == DetectionMode.BALL,
                    onClick = { onDetectionModeChange(DetectionMode.BALL) },
                    label = { Text("Ball") }
                )
            }
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                "Ball mode tracks a yellow tennis ball by colour (works through a net), falling back " +
                    "to the COCO \"sports ball\" model -- " +
                    "use it for the pendulum test in TESTING.md.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }

        Spacer(modifier = Modifier.height(16.dp))

        SectionCard(title = "Test Run") {
            OutlinedTextField(
                value = testName,
                onValueChange = onTestNameChange,
                label = { Text("Test name") },
                modifier = Modifier.fillMaxWidth(),
                singleLine = true,
                colors = OutlinedTextFieldDefaults.colors(
                    focusedTextColor = MaterialTheme.colorScheme.onSurface,
                    unfocusedTextColor = MaterialTheme.colorScheme.onSurface
                )
            )
            Spacer(modifier = Modifier.height(8.dp))
            OutlinedTextField(
                value = testNotes,
                onValueChange = onTestNotesChange,
                label = { Text("Notes (e.g. pendulum length/angle)") },
                modifier = Modifier.fillMaxWidth(),
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Text),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedTextColor = MaterialTheme.colorScheme.onSurface,
                    unfocusedTextColor = MaterialTheme.colorScheme.onSurface
                )
            )
            Spacer(modifier = Modifier.height(12.dp))
            Button(
                onClick = { onToggleLogging(!isLogging) },
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (isLogging) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.primary
                ),
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(if (isLogging) "Stop Logging" else "Start Logging")
            }
            Spacer(modifier = Modifier.height(8.dp))
            Text(
                "The gains and test name/notes above are written as a comment header " +
                    "in the saved CSV, and each row now includes ErrX/ErrY (tracking " +
                    "error from frame centre). See TESTING.md for suggested experiments, " +
                    "including a pendulum test that sweeps swing height/velocity.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }

        Spacer(modifier = Modifier.height(24.dp))
        Button(onClick = onBack, modifier = Modifier.fillMaxWidth()) { Text("Back") }
        Spacer(modifier = Modifier.height(16.dp))
    }
}

@Composable
private fun SectionCard(title: String, content: @Composable ColumnScope.() -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.onSurface)
            Spacer(modifier = Modifier.height(8.dp))
            content()
        }
    }
}

@Composable
private fun GainSlider(
    label: String,
    value: Float,
    range: ClosedFloatingPointRange<Float>,
    decimals: Int = 1,
    onValueChange: (Float) -> Unit
) {
    Column(modifier = Modifier.fillMaxWidth()) {
        Text(
            "$label: ${String.format(Locale.US, "%.${decimals}f", value)}",
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurface
        )
        Slider(
            value = value,
            onValueChange = onValueChange,
            valueRange = range
        )
    }
}
