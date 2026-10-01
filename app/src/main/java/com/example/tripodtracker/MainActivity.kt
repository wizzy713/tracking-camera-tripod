package com.example.tripodtracker

import android.Manifest
import android.content.ContentValues
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Rect
import android.os.Build
import android.os.Bundle
import android.provider.MediaStore
import android.util.Log
import android.view.WindowManager
import android.os.SystemClock
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.BackHandler
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.annotation.OptIn
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.video.*
import androidx.camera.video.VideoCapture
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.example.tripodtracker.ui.theme.TripodTrackerTheme
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetector as BallDetector
import com.google.android.gms.tasks.Tasks
import com.google.mlkit.vision.common.InputImage
import com.google.mlkit.vision.face.FaceDetection
import com.google.mlkit.vision.face.FaceDetector
import com.google.mlkit.vision.face.FaceDetectorOptions
import kotlinx.coroutines.awaitCancellation
import kotlinx.coroutines.delay
import java.text.SimpleDateFormat
import java.util.*
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread
import kotlin.math.hypot

// How far ahead the Kalman filter predicts, in seconds, to compensate for
// motor + network latency. This is a placeholder -- measure real end-to-end
// latency (LED-flash + high-speed-camera test) and replace this with that value.
// Live-tunable from the Experiment tab (MainActivity.predictionHorizonSeconds);
// this is just the value it starts at.
private const val DEFAULT_PREDICTION_HORIZON_SECONDS = 0.2f

/**
 * Which pipeline is used to find the tracked subject. FACE (default) is the
 * existing ML Kit face-lock behavior. BALL finds an optic-yellow tennis ball by
 * colour (ColorBallDetector, robust to a net around it), falling back to
 * MediaPipe's Object Detector with an EfficientDet-Lite0 COCO model filtered to
 * the "sports ball" class, so only balls are tracked -- e.g. for the pendulum test in TESTING.md. Selectable from
 * the Experiment tab or the camera-screen toggle icon.
 */
enum class DetectionMode { FACE, BALL }

// Tripod address persistence (Settings screen -> SharedPreferences).
private const val PREFS_NAME = "tripod_connection"
private const val PREF_ESP32_IP = "esp32_ip"
private const val PREF_UDP_PORT = "udp_port"
private const val DEFAULT_ESP32_IP = "10.47.140.33"
private const val DEFAULT_UDP_PORT = 4210
// Auto-discovery: probe every DISCOVERY_INTERVAL_MS once the tripod has been
// silent for DISCOVERY_SILENCE_MS (its telemetry normally arrives every ~2 s).
private const val DISCOVERY_INTERVAL_MS = 2000L
private const val DISCOVERY_SILENCE_MS = 5000L

// COCO model bundled in assets/ for DetectionMode.BALL, and the only class kept.
private const val BALL_MODEL_ASSET = "efficientdet_lite0.tflite"
private const val BALL_CATEGORY = "sports ball"
// Balls are small and often motion-blurred at the bottom of a swing, so this is
// lower than MediaPipe's usual 0.5 default; raise it if false positives appear.
private const val BALL_SCORE_THRESHOLD = 0.3f

// How many consecutive frames a locked target may go unmatched before the lock
// is released. During this window the system coasts on the Kalman prediction
// instead of jumping to an arbitrary detection.
private const val MAX_COAST_FRAMES = 15

// Debug: feed the battery overlay a synthetic, slowly-draining value so it can
// be demoed without the INA219 hardware / firmware telemetry connected. When
// false (the normal state) the overlay shows a "no tripod data" placeholder
// until a real BATT packet arrives from UdpSender.onBattery.
private const val DEBUG_FAKE_BATTERY = false

// MediaPipe hand skeleton topology: index pairs into the 21-landmark hand model
// (0 = wrist, then thumb 1-4, index 5-8, middle 9-12, ring 13-16, pinky 17-20).
// Used to draw the bone segments of the hand-tracking overlay.
private val HAND_CONNECTIONS = listOf(
    0 to 1, 1 to 2, 2 to 3, 3 to 4,        // thumb
    0 to 5, 5 to 6, 6 to 7, 7 to 8,        // index
    5 to 9, 9 to 10, 10 to 11, 11 to 12,   // middle
    9 to 13, 13 to 14, 14 to 15, 15 to 16, // ring
    13 to 17, 17 to 18, 18 to 19, 19 to 20, // pinky
    0 to 17                                // palm base
)

class MainActivity : ComponentActivity() {

    private lateinit var cameraExecutor: ExecutorService
    private val kalmanFilterX = KalmanFilter()
    private val kalmanFilterY = KalmanFilter()
    private val udpSender = UdpSender()
    lateinit var logManager: LogManager
    @Volatile var handLandmarker: HandLandmarker? = null

    // Tripod address. Loaded from / saved to SharedPreferences (see onCreate and
    // the Settings screen's onConnect), so it survives app restarts; these are
    // only the first-run defaults.
    private var esp32Ip by mutableStateOf(DEFAULT_ESP32_IP)
    private var udpPort by mutableIntStateOf(DEFAULT_UDP_PORT)
    private var isLogging by mutableStateOf(false)
    private var currentScreen by mutableStateOf("camera")
    private var lockedId by mutableStateOf<Int?>(null)
    private var permissionsGranted by mutableStateOf(false)
    private var packetSeq = (System.currentTimeMillis() % 1_000_000_000L)
    private var batteryStatus by mutableStateOf<BatteryStatus?>(null)

    // Experiment tab state: live PID gains as last reported by the tripod (null
    // until the first CFG reply arrives), which subject-finding pipeline is
    // active, the Kalman prediction horizon, and free-text metadata for the CSV
    // session header -- see buildSessionMetadata() and ExperimentScreen.kt.
    private var tripodConfig by mutableStateOf<TripodConfig?>(null)
    private var detectionMode by mutableStateOf(DetectionMode.FACE)
    private var predictionHorizonSeconds by mutableStateOf(DEFAULT_PREDICTION_HORIZON_SECONDS)
    private var testName by mutableStateOf("")
    private var testNotes by mutableStateOf("")

    data class DetectedObjectInfo(
        val boundingBox: Rect,
        val trackingId: Int?,
        val label: String = "Object",
        val isLocked: Boolean = false
    )

    data class DetectionResult(
        val objects: List<DetectedObjectInfo>,
        val imageWidth: Int,
        val imageHeight: Int,
        val isFrontCamera: Boolean,
        // Normalized [0,1] hand landmarks in the upright frame, for the skeleton
        // overlay drawn during the lock-on gesture. Empty when no hand is seen.
        val handLandmarks: List<Offset> = emptyList(),
        // True only on frames where the hand detector actually ran (it runs
        // every Nth frame); lets the UI refresh the overlay without flicker on
        // the skipped frames and clear it once the hand truly leaves.
        val handChecked: Boolean = false
    )

    /** Latest hand skeleton to render. Landmarks are normalized [0,1] in the
     *  upright frame; the preview fills the screen so they map straight onto it
     *  (mirrored for the front camera, like the bounding box). */
    data class HandOverlay(
        val landmarks: List<Offset>,
        val isFrontCamera: Boolean
    )

    /**
     * One processed frame's worth of tracking output: the normalized error sent
     * to the tripod, plus enough raw/filtered state to reconstruct any of the
     * evaluation plots (ablation, prediction-horizon sweep, packet loss) offline.
     */
    data class TrackingUpdate(
        val errX: Float,
        val errY: Float,
        val frameTimestampNanos: Long,
        val detectionCount: Int,
        val rawX: Float,
        val rawY: Float,
        val filteredX: Float,
        val filteredY: Float,
        val velocityX: Float,
        val velocityY: Float,
        val dtSeconds: Float
    )

    private val permissions = if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
        arrayOf(
            Manifest.permission.CAMERA,
            Manifest.permission.RECORD_AUDIO,
            Manifest.permission.WRITE_EXTERNAL_STORAGE
        )
    } else {
        arrayOf(
            Manifest.permission.CAMERA,
            Manifest.permission.RECORD_AUDIO
        )
    }

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { results ->
        permissionsGranted = results[Manifest.permission.CAMERA] == true
        if (!permissionsGranted) {
            Toast.makeText(this, "Camera permission is required.", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        cameraExecutor = Executors.newSingleThreadExecutor()
        logManager = LogManager(this)

        val prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
        esp32Ip = prefs.getString(PREF_ESP32_IP, null)?.takeIf { it.isNotBlank() } ?: DEFAULT_ESP32_IP
        udpPort = prefs.getInt(PREF_UDP_PORT, DEFAULT_UDP_PORT).takeIf { it in 1..65535 } ?: DEFAULT_UDP_PORT

        // Battery telemetry from the tripod's INA219 fuel gauge (UDP reply).
        udpSender.onBattery = { status ->
            Log.d("CamX", "battery telemetry: $status")
            runOnUiThread { batteryStatus = status }
        }

        // PID config replies from the tripod (Experiment tab "Sync"/"Apply").
        udpSender.onConfig = { config ->
            Log.d("CamX", "tripod config: $config")
            runOnUiThread { tripodConfig = config }
        }
        // Auto-connect: adopt (and save) the address of whichever tripod answers
        // a DISCOVER broadcast, if it isn't the one we're already using.
        udpSender.onTripodFound = { ip ->
            runOnUiThread {
                if (ip != esp32Ip && !isFinishing && !isDestroyed) {
                    Log.i("CamX", "tripod discovered at $ip (was $esp32Ip)")
                    esp32Ip = ip
                    saveConnection(ip, udpPort)
                    Toast.makeText(this, "Tripod found at $ip", Toast.LENGTH_SHORT).show()
                }
            }
        }

        thread { setupHandLandmarker() }

        permissionsGranted = allPermissionsGranted()
        setContent {
            TripodTrackerTheme {
                if (permissionsGranted) {
                    CamXApp()
                } else {
                    PermissionRequestScreen(onRequestPermission = { requestPermissionLauncher.launch(permissions) })
                }
            }
        }
        if (!permissionsGranted) {
            requestPermissionLauncher.launch(permissions)
        }
    }

    private fun setupHandLandmarker() {
        try {
            val baseOptions = BaseOptions.builder()
                .setModelAssetPath("hand_landmarker.task")
                .build()
            val options = HandLandmarker.HandLandmarkerOptions.builder()
                .setBaseOptions(baseOptions)
                .setMinHandDetectionConfidence(0.5f)
                .setMinTrackingConfidence(0.5f)
                .setMinHandPresenceConfidence(0.5f)
                .setNumHands(1)
                .setRunningMode(RunningMode.IMAGE)
                .build()
            handLandmarker = HandLandmarker.createFromOptions(this, options)
            Log.d("CamX", "HandLandmarker initialized successfully")
        } catch (e: Exception) {
            Log.e("CamX", "HandLandmarker init failed: ${e.message}")
        }
    }

    private fun allPermissionsGranted() = permissions.all {
        ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
    }

    @Composable
    fun CamXApp() {
        LaunchedEffect(esp32Ip, udpPort) {
            udpSender.updateTarget(esp32Ip, udpPort)
        }

        // While nothing has been heard from the tripod for a few seconds (it
        // pushes telemetry every ~2 s once connected), broadcast DISCOVER so a
        // tripod with a new DHCP address is found and adopted automatically.
        LaunchedEffect(Unit) {
            while (true) {
                delay(DISCOVERY_INTERVAL_MS)
                if (SystemClock.elapsedRealtime() - udpSender.lastRxFromTargetMs > DISCOVERY_SILENCE_MS) {
                    udpSender.discover()
                }
            }
        }

        if (DEBUG_FAKE_BATTERY) {
            LaunchedEffect(Unit) {
                var pct = (62..96).random()
                while (true) {
                    batteryStatus = BatteryStatus(
                        percent = pct,
                        // ~6.0 V empty .. ~8.4 V full for a 2S pack.
                        millivolts = (6000 + pct * 24).coerceIn(6000, 8400),
                        milliamps = (220..880).random(),
                        whRemaining = pct / 100f * 9.62f
                    )
                    delay(3000)
                    pct -= (0..1).random()
                    if (pct < 12) pct = (86..97).random() // loop for a continuous demo
                }
            }
        }

        val onToggleLogging: (Boolean) -> Unit = {
            isLogging = it
            if (it) logManager.startSession(buildSessionMetadata()) else logManager.saveLog()
        }

        when (currentScreen) {
            "settings" -> {
                BackHandler { currentScreen = "camera" }
                ConnectionScreen(
                    currentIp = esp32Ip,
                    currentPort = udpPort,
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    onConnect = { ip, port ->
                        esp32Ip = ip
                        udpPort = port
                        saveConnection(ip, port)
                        currentScreen = "camera"
                    },
                    onTest = { ip, port, msg ->
                        udpSender.updateTarget(ip, port)
                        udpSender.send(msg)
                        Toast.makeText(this, "Test packet sent to $ip", Toast.LENGTH_SHORT).show()
                    }
                )
            }
            "experiment" -> {
                BackHandler { currentScreen = "camera" }
                ExperimentScreen(
                    tripodConfig = tripodConfig,
                    onApplyPid = { config ->
                        udpSender.send(
                            String.format(
                                Locale.US,
                                "CFG:KP:%.2f,KI:%.2f,KD:%.2f,MS:%.1f,DZ:%.3f",
                                config.kp, config.ki, config.kd, config.maxSpeedOffsetUs, config.deadzone
                            )
                        )
                    },
                    onSyncFromTripod = { udpSender.send("CFG?") },
                    kalmanFilterX = kalmanFilterX,
                    kalmanFilterY = kalmanFilterY,
                    predictionHorizonSeconds = predictionHorizonSeconds,
                    onPredictionHorizonChange = { predictionHorizonSeconds = it },
                    detectionMode = detectionMode,
                    onDetectionModeChange = { mode ->
                        detectionMode = mode
                        lockedId = null
                        kalmanFilterX.reset()
                        kalmanFilterY.reset()
                    },
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    testName = testName,
                    onTestNameChange = { testName = it },
                    testNotes = testNotes,
                    onTestNotesChange = { testNotes = it },
                    onBack = { currentScreen = "camera" }
                )
            }
            else -> {
                CameraPreviewScreen(
                    cameraExecutor,
                    kalmanFilterX,
                    kalmanFilterY,
                    handLandmarker,
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    onOpenSettings = { currentScreen = "settings" },
                    onOpenExperiment = { currentScreen = "experiment" },
                    lockedId = lockedId,
                    battery = batteryStatus,
                    detectionMode = detectionMode,
                    onDetectionModeChange = { mode ->
                        detectionMode = mode
                        lockedId = null
                        kalmanFilterX.reset()
                        kalmanFilterY.reset()
                    },
                    predictionHorizonSeconds = predictionHorizonSeconds,
                    onUnlock = { lockedId = null },
                    onTargetUpdate = { id -> lockedId = id }
                ) { update ->
                    val seq = packetSeq++
                    udpSender.send(String.format(Locale.US, "EX:%.4f,EY:%.4f,SEQ:%d", update.errX, update.errY, seq))
                    if (isLogging) {
                        logManager.log(
                            frameTimestampNanos = update.frameTimestampNanos,
                            seq = seq,
                            detectionCount = update.detectionCount,
                            rawX = update.rawX,
                            rawY = update.rawY,
                            filteredX = update.filteredX,
                            filteredY = update.filteredY,
                            velocityX = update.velocityX,
                            velocityY = update.velocityY,
                            dtSeconds = update.dtSeconds,
                            errX = update.errX,
                            errY = update.errY
                        )
                    }
                }
            }
        }
    }

    /**
     * Snapshot of every tunable in effect right now, written as a CSV comment
     * header by LogManager.saveLog() so a saved log is self-describing about
     * which gains produced it. Called when logging is toggled ON.
     */
    private fun saveConnection(ip: String, port: Int) {
        getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit()
            .putString(PREF_ESP32_IP, ip)
            .putInt(PREF_UDP_PORT, port)
            .apply()
    }

    private fun buildSessionMetadata(): Map<String, String> {
        val cfg = tripodConfig
        return linkedMapOf(
            "test_name" to testName.ifBlank { "(unnamed)" },
            "test_notes" to testNotes,
            "detection_mode" to detectionMode.name,
            "prediction_horizon_s" to predictionHorizonSeconds.toString(),
            "kalman_measurement_noise" to kalmanFilterX.measurementNoise.toString(),
            "kalman_acceleration_noise" to kalmanFilterX.accelerationNoise.toString(),
            "tripod_kp" to (cfg?.kp?.toString() ?: "unknown (not synced)"),
            "tripod_ki" to (cfg?.ki?.toString() ?: "unknown (not synced)"),
            "tripod_kd" to (cfg?.kd?.toString() ?: "unknown (not synced)"),
            "tripod_max_speed_offset_us" to (cfg?.maxSpeedOffsetUs?.toString() ?: "unknown (not synced)"),
            "tripod_deadzone" to (cfg?.deadzone?.toString() ?: "unknown (not synced)")
        )
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraExecutor.shutdown()
        try {
            cameraExecutor.awaitTermination(2, TimeUnit.SECONDS)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        handLandmarker?.close()
        udpSender.close()
    }
}

@Composable
fun PermissionRequestScreen(onRequestPermission: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize().background(MaterialTheme.colorScheme.background).padding(24.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center
    ) {
        Text(
            "Camera Permission Required",
            style = MaterialTheme.typography.headlineSmall,
            color = MaterialTheme.colorScheme.onBackground
        )
        Spacer(modifier = Modifier.height(16.dp))
        Text(
            "CamX needs camera access to track and follow your subject.",
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onBackground
        )
        Spacer(modifier = Modifier.height(24.dp))
        Button(onClick = onRequestPermission) { Text("Grant Permission") }
    }
}

@Composable
fun CameraPreviewScreen(
    executor: ExecutorService,
    kalmanFilterX: KalmanFilter,
    kalmanFilterY: KalmanFilter,
    landmarker: HandLandmarker?,
    isLogging: Boolean,
    onToggleLogging: (Boolean) -> Unit,
    onOpenSettings: () -> Unit,
    onOpenExperiment: () -> Unit,
    lockedId: Int?,
    battery: BatteryStatus?,
    detectionMode: DetectionMode,
    onDetectionModeChange: (DetectionMode) -> Unit,
    predictionHorizonSeconds: Float,
    onUnlock: () -> Unit,
    onTargetUpdate: (Int?) -> Unit,
    onTargetDetected: (MainActivity.TrackingUpdate) -> Unit
) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val previewView = remember { PreviewView(context) }

    // The analyzer lambda below is created once per camera bind, so reading the
    // plain parameters there would freeze them at bind time (e.g. the mode toggle
    // or a palm lock wouldn't reach the pipeline until the camera rebinds). These
    // always hold the latest recomposed values.
    val currentLandmarker by rememberUpdatedState(landmarker)
    val currentLockedId by rememberUpdatedState(lockedId)
    val currentDetectionMode by rememberUpdatedState(detectionMode)
    val currentPredictionHorizon by rememberUpdatedState(predictionHorizonSeconds)

    var cameraSelector by remember { mutableStateOf(CameraSelector.DEFAULT_BACK_CAMERA) }
    var detectionResult by remember { mutableStateOf<MainActivity.DetectionResult?>(null) }
    var handOverlay by remember { mutableStateOf<MainActivity.HandOverlay?>(null) }
    var isTrackingEnabled by remember { mutableStateOf(true) }
    var flashMode by remember { mutableIntStateOf(ImageCapture.FLASH_MODE_OFF) }

    val imageCapture = remember { ImageCapture.Builder().setFlashMode(flashMode).build() }
    val recorder = remember { Recorder.Builder().setQualitySelector(QualitySelector.from(Quality.HIGHEST)).build() }
    val videoCapture = remember { VideoCapture.withOutput(recorder) }
    var activeRecording by remember { mutableStateOf<Recording?>(null) }

    val cameraProviderFuture = remember { ProcessCameraProvider.getInstance(context) }

    Box(modifier = Modifier.fillMaxSize()) {
        AndroidView(factory = { previewView }, modifier = Modifier.fillMaxSize())

        if (isTrackingEnabled) {
            Text(
                text = if (lockedId != null) "LOCKED" else if (detectionResult != null) "Tracking" else "Searching",
                color = if (lockedId != null) Color.Cyan else Color.Green,
                modifier = Modifier.padding(top = 64.dp).align(Alignment.TopCenter),
                style = MaterialTheme.typography.headlineSmall
            )

            Canvas(modifier = Modifier.fillMaxSize()) {
                detectionResult?.let { result ->
                    val isFront = result.isFrontCamera
                    val scaleX = size.width / result.imageWidth
                    val scaleY = size.height / result.imageHeight

                    // Only draw the target object (Locked one, or the primary one)
                    val targetObj = if (lockedId != null) {
                        result.objects.find { it.trackingId == lockedId }
                    } else {
                        result.objects.firstOrNull()
                    }

                    targetObj?.let { obj ->
                        val left = if (isFront) size.width - (obj.boundingBox.right * scaleX) else obj.boundingBox.left * scaleX
                        val right = if (isFront) size.width - (obj.boundingBox.left * scaleX) else obj.boundingBox.right * scaleX
                        val top = obj.boundingBox.top * scaleY
                        val bottom = obj.boundingBox.bottom * scaleY

                        drawRect(
                            color = if (obj.isLocked) Color.Cyan else Color.Green,
                            topLeft = Offset(left, top),
                            size = Size(right - left, bottom - top),
                            style = Stroke(width = if (obj.isLocked) 8.dp.toPx() else 6.dp.toPx())
                        )
                    }
                }

                // Hand skeleton overlay -- drawn while the lock-on gesture is
                // available (i.e. not yet locked) so the user can see the hand
                // being recognized before the open palm picks a subject.
                if (lockedId == null) {
                    handOverlay?.let { hand ->
                        if (hand.landmarks.size >= 21) {
                            val pts = hand.landmarks.map { lm ->
                                val x = lm.x * size.width
                                Offset(if (hand.isFrontCamera) size.width - x else x, lm.y * size.height)
                            }
                            HAND_CONNECTIONS.forEach { (a, b) ->
                                drawLine(
                                    color = Color(0xFF00E5FF),
                                    start = pts[a],
                                    end = pts[b],
                                    strokeWidth = 3.dp.toPx()
                                )
                            }
                            pts.forEach { p ->
                                drawCircle(color = Color.White, radius = 4.dp.toPx(), center = p)
                            }
                        }
                    }
                }
            }
        }

        Row(
            modifier = Modifier.fillMaxWidth().padding(16.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row {
                IconButton(onClick = onOpenSettings) {
                    Icon(Icons.Default.Settings, contentDescription = "Settings", tint = Color.White)
                }
                IconButton(onClick = onOpenExperiment) {
                    Icon(Icons.Default.Tune, contentDescription = "Experiment", tint = Color.White)
                }
                IconButton(onClick = {
                    val next = if (detectionMode == DetectionMode.FACE) DetectionMode.BALL else DetectionMode.FACE
                    onDetectionModeChange(next)
                }) {
                    Icon(
                        imageVector = if (detectionMode == DetectionMode.FACE) Icons.Default.Face else Icons.Default.SportsBaseball,
                        contentDescription = "Detection mode: ${detectionMode.name}",
                        tint = Color.White
                    )
                }
            }

            Row {
                if (lockedId != null) {
                    IconButton(onClick = onUnlock) {
                        Icon(Icons.Default.LockOpen, contentDescription = "Unlock", tint = Color.Cyan)
                    }
                }

                IconButton(onClick = {
                    isTrackingEnabled = !isTrackingEnabled
                    if (!isTrackingEnabled) {
                        onUnlock()
                        kalmanFilterX.reset()
                        kalmanFilterY.reset()
                        onTargetDetected(
                            MainActivity.TrackingUpdate(
                                errX = 0f, errY = 0f,
                                frameTimestampNanos = System.nanoTime(),
                                detectionCount = 0,
                                rawX = Float.NaN, rawY = Float.NaN,
                                filteredX = 0f, filteredY = 0f,
                                velocityX = 0f, velocityY = 0f,
                                dtSeconds = 0f
                            )
                        )
                    }
                }) {
                    Icon(
                        imageVector = if (isTrackingEnabled) Icons.Filled.TrackChanges else Icons.Filled.LocationDisabled,
                        contentDescription = "Tracking",
                        tint = if (isTrackingEnabled) Color.Green else Color.White
                    )
                }

                IconButton(onClick = { onToggleLogging(!isLogging) }) {
                    Icon(
                        imageVector = if (isLogging) Icons.Default.Save else Icons.Default.Description,
                        contentDescription = "Log",
                        tint = if (isLogging) Color.Red else Color.White
                    )
                }

                IconButton(onClick = {
                    cameraSelector = if (cameraSelector == CameraSelector.DEFAULT_BACK_CAMERA) CameraSelector.DEFAULT_FRONT_CAMERA else CameraSelector.DEFAULT_BACK_CAMERA
                    onUnlock()
                    kalmanFilterX.reset()
                    kalmanFilterY.reset()
                }) {
                    Icon(Icons.Filled.FlipCameraAndroid, contentDescription = "Flip", tint = Color.White)
                }
            }
        }

        BatteryIndicator(
            battery = battery,
            modifier = Modifier.align(Alignment.TopEnd).padding(top = 72.dp, end = 16.dp)
        )

        Row(
            modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 48.dp).fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceEvenly,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier.size(64.dp).clip(CircleShape).background(if (activeRecording != null) Color.Red else Color.White.copy(alpha = 0.5f)).border(2.dp, Color.White, CircleShape).clickable {
                    val recording = activeRecording
                    if (recording != null) {
                        recording.stop()
                        activeRecording = null
                    } else {
                        activeRecording = startVideoRecording(context, videoCapture, executor)
                    }
                },
                contentAlignment = Alignment.Center
            ) {
                Icon(
                    imageVector = if (activeRecording != null) Icons.Filled.Stop else Icons.Filled.Videocam,
                    contentDescription = "Record",
                    tint = if (activeRecording != null) Color.White else Color.Black
                )
            }

            Box(
                modifier = Modifier.size(80.dp).clip(CircleShape).background(Color.White).border(4.dp, Color.Gray, CircleShape).clickable { takePhoto(context, imageCapture, executor) },
                contentAlignment = Alignment.Center
            ) {
                Icon(Icons.Filled.CameraAlt, contentDescription = "Capture", tint = Color.Black, modifier = Modifier.size(40.dp))
            }
        }

        LaunchedEffect(cameraSelector, isTrackingEnabled) {
            val cameraProvider = cameraProviderFuture.get()
            val preview = Preview.Builder().build().also { it.setSurfaceProvider(previewView.surfaceProvider) }

            val faceOptions = FaceDetectorOptions.Builder()
                .setPerformanceMode(FaceDetectorOptions.PERFORMANCE_MODE_FAST)
                .enableTracking()
                .build()
            val faceDetector = FaceDetection.getClient(faceOptions)

            // Ball detector for DetectionMode.BALL. Created lazily on the analysis
            // executor the first time BALL mode sees a frame (model load is too slow
            // for the main thread, and FACE-only sessions never pay for it), and only
            // ever used and closed on that same thread.
            var ballDetector: BallDetector? = null
            // Set (on the executor) once this effect is torn down. A frame already
            // queued for this effect's analyzer can still run after the close task,
            // and calling detect() on a closed MediaPipe detector is a native
            // SIGSEGV, not a catchable exception -- so never hand one out.
            var ballDetectorClosed = false
            val ballTracker = BallTracker()
            val colorBallDetector = ColorBallDetector()
            val getBallDetector: () -> BallDetector? = {
                if (ballDetector == null && !ballDetectorClosed) {
                    try {
                        val options = BallDetector.ObjectDetectorOptions.builder()
                            .setBaseOptions(BaseOptions.builder().setModelAssetPath(BALL_MODEL_ASSET).build())
                            .setRunningMode(RunningMode.IMAGE)
                            .setCategoryAllowlist(listOf(BALL_CATEGORY))
                            .setScoreThreshold(BALL_SCORE_THRESHOLD)
                            .setMaxResults(5)
                            .build()
                        ballDetector = BallDetector.createFromOptions(context, options)
                    } catch (e: Exception) {
                        Log.e("CamX", "Ball detector init failed: ${e.message}")
                    }
                }
                ballDetector
            }

            var frameCounter = 0
            var lostFrameCount = 0
            val imageAnalyzer = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                .build()
                .also {
                it.setAnalyzer(executor) { imageProxy ->
                    if (isTrackingEnabled) {
                        frameCounter++
                        processImageProxy(
                            faceDetector,
                            getBallDetector,
                            colorBallDetector,
                            ballTracker,
                            currentDetectionMode,
                            currentLandmarker,
                            imageProxy,
                            cameraSelector == CameraSelector.DEFAULT_FRONT_CAMERA,
                            kalmanFilterX,
                            kalmanFilterY,
                            currentPredictionHorizon,
                            currentLockedId,
                            lostFrameCount,
                            // Skip the (expensive: JPEG round-trip + inference) hand
                            // landmark pass entirely once a target is locked -- its only
                            // consumers (the skeleton overlay and the open-palm lock
                            // gesture) are both gated on lockedId == null already, so
                            // running it while locked only steals executor time from
                            // the per-frame tracking pipeline and adds latency to the
                            // servo correction.
                            frameCounter % 5 == 0 && currentLockedId == null,
                            onTargetUpdate,
                            onLostFrameCountChanged = { lostFrameCount = it }
                        ) { update, result ->
                            onTargetDetected(update)
                            detectionResult = if (result.objects.isEmpty()) null else result
                            if (result.handChecked) {
                                handOverlay = if (result.handLandmarks.size >= 21) {
                                    MainActivity.HandOverlay(
                                        landmarks = result.handLandmarks,
                                        isFrontCamera = result.isFrontCamera
                                    )
                                } else null
                            }
                        }
                    } else {
                        imageProxy.close()
                    }
                }
            }

            try {
                cameraProvider.unbindAll()
                cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview, imageAnalyzer, imageCapture, videoCapture)
            } catch (exc: Exception) {
                Log.e("CameraX", "Full use-case binding failed, retrying without video capture", exc)
                try {
                    cameraProvider.unbindAll()
                    cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview, imageAnalyzer, imageCapture)
                } catch (exc2: Exception) {
                    Log.e("CameraX", "Binding failed even without video capture", exc2)
                }
            }

            try {
                awaitCancellation()
            } finally {
                // Stop new frames reaching this effect's analyzer before closing the
                // detectors it uses; the close runs on the executor, after any frame
                // already in flight there.
                imageAnalyzer.clearAnalyzer()
                faceDetector.close()
                executor.execute {
                    ballDetectorClosed = true
                    ballDetector?.close()
                    ballDetector = null
                }
            }
        }
    }
}

/**
 * Compact battery readout for the camera overlay: a fill-level glyph plus the
 * percentage and pack voltage from the tripod's INA219 fuel gauge. Colour tracks
 * charge (green > 50%, amber 20-50%, red < 20%). Until the first telemetry
 * packet arrives it shows a dim "no data" placeholder, so the feature is
 * visibly present even before the tripod is connected.
 */
@Composable
fun BatteryIndicator(battery: BatteryStatus?, modifier: Modifier = Modifier) {
    val levelColor = when {
        battery == null -> Color.White.copy(alpha = 0.5f)
        battery.milliamps < -20 -> Color(0xFF4CAF50) // negative current => charging
        battery.percent > 50 -> Color(0xFF4CAF50)
        battery.percent > 20 -> Color(0xFFFFB300)
        else -> Color(0xFFE53935)
    }

    Row(
        modifier = modifier
            .clip(RoundedCornerShape(10.dp))
            .background(Color.Black.copy(alpha = 0.45f))
            .padding(horizontal = 8.dp, vertical = 5.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Canvas(modifier = Modifier.size(width = 26.dp, height = 13.dp)) {
            val stroke = 1.5.dp.toPx()
            val nubW = 2.dp.toPx()
            val bodyW = size.width - nubW
            val corner = CornerRadius(2.dp.toPx(), 2.dp.toPx())
            val outline = if (battery == null) Color.White.copy(alpha = 0.5f) else Color.White

            drawRoundRect(
                color = outline,
                topLeft = Offset(0f, 0f),
                size = Size(bodyW, size.height),
                cornerRadius = corner,
                style = Stroke(width = stroke)
            )
            drawRoundRect(
                color = outline,
                topLeft = Offset(bodyW, size.height * 0.28f),
                size = Size(nubW, size.height * 0.44f),
                cornerRadius = CornerRadius(1.dp.toPx(), 1.dp.toPx())
            )
            if (battery != null) {
                val pad = stroke + 1.dp.toPx()
                val trackW = bodyW - 2 * pad
                val fillW = (trackW * (battery.percent / 100f)).coerceIn(0f, trackW)
                drawRoundRect(
                    color = levelColor,
                    topLeft = Offset(pad, pad),
                    size = Size(fillW, size.height - 2 * pad),
                    cornerRadius = CornerRadius(1.dp.toPx(), 1.dp.toPx())
                )
            }
        }

        Spacer(modifier = Modifier.width(6.dp))

        Column {
            Text(
                text = if (battery != null) "${battery.percent}%" else "-- %",
                color = if (battery != null) Color.White else Color.White.copy(alpha = 0.6f),
                style = MaterialTheme.typography.labelLarge
            )
            Text(
                text = if (battery != null) {
                    String.format(Locale.US, "%.2f V", battery.millivolts / 1000f)
                } else {
                    "no tripod data"
                },
                color = Color.White.copy(alpha = 0.6f),
                style = MaterialTheme.typography.labelSmall
            )
        }
    }
}

@Composable
fun ConnectionScreen(
    currentIp: String,
    currentPort: Int,
    isLogging: Boolean,
    onToggleLogging: (Boolean) -> Unit,
    onConnect: (String, Int) -> Unit,
    onTest: (String, Int, String) -> Unit
) {
    var ip by remember { mutableStateOf(currentIp) }
    var port by remember { mutableStateOf(currentPort.toString()) }
    var testMessage by remember { mutableStateOf("PING") }

    Column(
        modifier = Modifier.fillMaxSize().background(MaterialTheme.colorScheme.background).padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("Tripod Connection", style = MaterialTheme.typography.headlineMedium, color = MaterialTheme.colorScheme.onBackground)
        Spacer(modifier = Modifier.height(24.dp))

        OutlinedTextField(
            value = ip,
            onValueChange = { ip = it },
            label = { Text("ESP32 IP Address") },
            modifier = Modifier.fillMaxWidth(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
            colors = OutlinedTextFieldDefaults.colors(focusedTextColor = MaterialTheme.colorScheme.onBackground, unfocusedTextColor = MaterialTheme.colorScheme.onBackground)
        )

        Spacer(modifier = Modifier.height(8.dp))

        OutlinedTextField(
            value = port,
            onValueChange = { port = it },
            label = { Text("UDP Port") },
            modifier = Modifier.fillMaxWidth(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            colors = OutlinedTextFieldDefaults.colors(focusedTextColor = MaterialTheme.colorScheme.onBackground, unfocusedTextColor = MaterialTheme.colorScheme.onBackground)
        )

        Spacer(modifier = Modifier.height(8.dp))

        OutlinedTextField(
            value = testMessage,
            onValueChange = { testMessage = it },
            label = { Text("Custom Test Message") },
            modifier = Modifier.fillMaxWidth(),
            colors = OutlinedTextFieldDefaults.colors(focusedTextColor = MaterialTheme.colorScheme.onBackground, unfocusedTextColor = MaterialTheme.colorScheme.onBackground)
        )

        Spacer(modifier = Modifier.height(16.dp))

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            // Trim and range-check here so a stray space or bad port is never
            // saved to preferences (and never reaches DatagramPacket).
            val cleanIp = ip.trim()
            val cleanPort = port.trim().toIntOrNull()?.takeIf { it in 1..65535 } ?: DEFAULT_UDP_PORT
            Button(onClick = { onTest(cleanIp, cleanPort, testMessage) }, enabled = cleanIp.isNotEmpty()) { Text("Test Connection") }
            Button(onClick = { onConnect(cleanIp, cleanPort) }, enabled = cleanIp.isNotEmpty()) { Text("Save & Connect") }
        }

        Spacer(modifier = Modifier.height(16.dp))

        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = if (isLogging) Color.Red.copy(alpha = 0.1f) else MaterialTheme.colorScheme.surface)
        ) {
            Row(modifier = Modifier.padding(16.dp).fillMaxWidth(), verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.SpaceBetween) {
                Text("CSV Logging", style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.onSurface)
                Button(onClick = { onToggleLogging(!isLogging) }, colors = ButtonDefaults.buttonColors(containerColor = if (isLogging) Color.Red else MaterialTheme.colorScheme.primary)) {
                    Text(if (isLogging) "Stop Logging" else "Start Logging")
                }
            }
        }
    }
}

@OptIn(ExperimentalGetImage::class)
private fun processImageProxy(
    faceDetector: FaceDetector,
    getBallDetector: () -> BallDetector?,
    colorBallDetector: ColorBallDetector,
    ballTracker: BallTracker,
    mode: DetectionMode,
    handLandmarker: HandLandmarker?,
    imageProxy: ImageProxy,
    isFrontCamera: Boolean,
    kalmanFilterX: KalmanFilter,
    kalmanFilterY: KalmanFilter,
    predictionHorizonSeconds: Float,
    lockedId: Int?,
    lostFrameCount: Int,
    shouldDetectHands: Boolean,
    onSetLockedId: (Int?) -> Unit,
    onLostFrameCountChanged: (Int) -> Unit,
    onResult: (MainActivity.TrackingUpdate, MainActivity.DetectionResult) -> Unit
) {
    val mediaImage = imageProxy.image
    if (mediaImage != null) {
        val rotation = imageProxy.imageInfo.rotationDegrees
        val image = InputImage.fromMediaImage(mediaImage, rotation)

        // ML Kit returns boxes in the upright (post-rotation) frame. In portrait
        // (rotation 90/270) that frame is taller-than-wide even though the raw
        // buffer is landscape, so width/height must be swapped to match.
        val frameWidth = if (rotation == 90 || rotation == 270) imageProxy.height else imageProxy.width
        val frameHeight = if (rotation == 90 || rotation == 270) imageProxy.width else imageProxy.height

        var openPalmDetected = false
        var palmX = 0f
        var palmY = 0f
        var handLandmarks: List<Offset> = emptyList()

        // Upright RGB copy of the frame for the MediaPipe models (hand landmarker
        // and ball detector). Lazy so it's built at most once per frame, and not
        // at all on FACE-mode frames that skip the hand pass.
        val uprightBitmap by lazy { imageProxy.toBitmapInternal(rotation) }

        // Only detect hands periodically to save resources and prevent crashes
        if (shouldDetectHands) {
            handLandmarker?.let { landmarker ->
                try {
                    // Rotate to the same upright frame ML Kit uses, so palm
                    // coordinates and bounding-box coordinates are comparable.
                    val mpImage = BitmapImageBuilder(uprightBitmap).build()
                    val result = landmarker.detect(mpImage)
                    if (result.landmarks().isNotEmpty()) {
                        val hand = result.landmarks()[0]
                        // Keep the full 21-point skeleton (normalized) for the
                        // overlay, regardless of pose.
                        handLandmarks = hand.map { Offset(it.x(), it.y()) }
                        val isExtended = hand[8].y() < hand[6].y() && hand[12].y() < hand[10].y() &&
                                        hand[16].y() < hand[14].y() && hand[20].y() < hand[18].y()
                        if (isExtended) {
                            openPalmDetected = true
                            palmX = hand[9].x() * frameWidth
                            palmY = hand[9].y() * frameHeight
                        }
                    }
                } catch (e: Exception) {
                    Log.e("CamX", "Hand detection error: ${e.message}")
                }
            }
        }

        // Map either detector's result list to a common shape so all the selection
        // logic below (largest-box, tracking-ID match, hand-lock distance) works
        // unchanged regardless of which pipeline is active. If the source task
        // failed, `t.result` throws here, which fails the mapped task the same way
        // the old code silently skipped a failed frame (no onSuccess, but
        // addOnCompleteListener below still closes imageProxy).
        val detectionTask = when (mode) {
            DetectionMode.FACE -> faceDetector.process(image).continueWith { t ->
                t.result.map { MainActivity.DetectedObjectInfo(it.boundingBox, it.trackingId) }
            }
            // Colour segmentation first: it finds the optic-yellow ball even through
            // the net holding it, where the COCO model mostly misses. The MediaPipe
            // model is only a fallback for frames with no yellow blob (e.g. a
            // different-coloured ball). Both run synchronously here; the result is
            // wrapped in a completed Task so it feeds the same listener chain as ML
            // Kit. Boxes are in the upright bitmap's frame (same as ML Kit's), and
            // IDs come from BallTracker.
            DetectionMode.BALL -> try {
                val boxes = colorBallDetector.detect(uprightBitmap).ifEmpty {
                    val detector = getBallDetector() ?: throw IllegalStateException("Ball detector unavailable")
                    detector.detect(BitmapImageBuilder(uprightBitmap).build()).detections().map { d ->
                        val b = d.boundingBox()
                        Rect(b.left.toInt(), b.top.toInt(), b.right.toInt(), b.bottom.toInt())
                    }
                }
                Tasks.forResult(ballTracker.update(boxes).map { (box, id) ->
                    MainActivity.DetectedObjectInfo(box, id, label = "Ball")
                })
            } catch (e: Exception) {
                Log.e("CamX", "Ball detection error: ${e.message}")
                Tasks.forException(e)
            }
        }

        detectionTask
            .addOnSuccessListener { detectedObjects ->
                if (openPalmDetected && lockedId == null) {
                    val closest = detectedObjects.minByOrNull { obj ->
                        hypot(obj.boundingBox.centerX().toFloat() - palmX, obj.boundingBox.centerY().toFloat() - palmY)
                    }
                    if (closest != null && closest.trackingId != null) {
                        onSetLockedId(closest.trackingId)
                    }
                }

                // Deterministic selection: largest box (by area) when unlocked, so the
                // target doesn't flicker between same-frame detections with no
                // guaranteed ordering. When locked, only match the tracked ID -- never
                // silently fall back to an arbitrary object.
                val targetObject = if (lockedId != null) {
                    detectedObjects.find { it.trackingId == lockedId }
                } else {
                    detectedObjects.maxByOrNull { it.boundingBox.width().toLong() * it.boundingBox.height().toLong() }
                }

                val objectInfos = targetObject?.let {
                    listOf(
                        MainActivity.DetectedObjectInfo(
                            it.boundingBox,
                            it.trackingId,
                            isLocked = it.trackingId == lockedId
                        )
                    )
                } ?: emptyList()

                val newLostFrameCount = if (lockedId != null && targetObject == null) {
                    (lostFrameCount + 1).also { if (it > MAX_COAST_FRAMES) onSetLockedId(null) }
                } else {
                    0
                }
                onLostFrameCountChanged(newLostFrameCount)

                val rawX: Float
                val rawY: Float
                val filteredX: Float
                val filteredY: Float

                // Coast on the Kalman prediction ONLY for a briefly-lost locked
                // target (short occlusion/misdetection). With no lock, or a lock
                // that has just expired, do not chase a subject we cannot see:
                // command "centered" and reset the filters. Sending a stale,
                // edge-clamped prediction frame after frame is exactly what turns
                // a momentary loss into a servo that spins until the app closes.
                val coasting = targetObject == null && lockedId != null &&
                        newLostFrameCount in 1..MAX_COAST_FRAMES

                val predictedX: Float
                val predictedY: Float

                if (targetObject != null) {
                    rawX = targetObject.boundingBox.exactCenterX()
                    rawY = targetObject.boundingBox.exactCenterY()
                    filteredX = kalmanFilterX.update(rawX, imageProxy.imageInfo.timestamp)
                    filteredY = kalmanFilterY.update(rawY, imageProxy.imageInfo.timestamp)
                    predictedX = kalmanFilterX.predictFuture(predictionHorizonSeconds)
                    predictedY = kalmanFilterY.predictFuture(predictionHorizonSeconds)
                } else if (coasting) {
                    rawX = Float.NaN
                    rawY = Float.NaN
                    filteredX = if (kalmanFilterX.hasEstimate) kalmanFilterX.position else frameWidth / 2f
                    filteredY = if (kalmanFilterY.hasEstimate) kalmanFilterY.position else frameHeight / 2f
                    predictedX = kalmanFilterX.predictFuture(predictionHorizonSeconds)
                    predictedY = kalmanFilterY.predictFuture(predictionHorizonSeconds)
                } else {
                    // Nothing to track -- hold still and forget the old trajectory.
                    rawX = Float.NaN
                    rawY = Float.NaN
                    kalmanFilterX.reset()
                    kalmanFilterY.reset()
                    filteredX = frameWidth / 2f
                    filteredY = frameHeight / 2f
                    predictedX = frameWidth / 2f
                    predictedY = frameHeight / 2f
                }

                // No front-camera mirroring here: CameraX's ImageAnalysis frame is the
                // raw, unmirrored sensor image for BOTH lenses (only the on-screen
                // PreviewView mirrors the front camera, as a display-only convention
                // for the user watching themselves). So predictedX already encodes the
                // subject's true physical position relative to the camera's boresight
                // for either camera, and the servo needs that -- not the mirrored,
                // "selfie view" coordinate. Flipping it here was inverting the pan
                // correction on the front camera. (The bounding-box/hand overlays
                // still mirror for isFrontCamera above -- that's a separate, correct
                // concern: drawing on top of the mirrored preview the user sees.)
                var errX = (predictedX - frameWidth / 2f) / (frameWidth / 2f)
                errX = errX.coerceIn(-1f, 1f)
                var errY = ((predictedY - frameHeight / 2f) / (frameHeight / 2f)).coerceIn(-1f, 1f)

                // Tilt sign, verified on the rig: the back camera needs the analysis
                // frame's Y flipped and the front camera (which faces the other way,
                // so the same tilt moves its view the opposite way) needs it as-is.
                // Both were previously the other way round, which drove tilt away
                // from the subject on both cameras.
                if (!isFrontCamera) {
                    errY = -errY
                }

                val update = MainActivity.TrackingUpdate(
                    errX = errX,
                    errY = errY,
                    frameTimestampNanos = imageProxy.imageInfo.timestamp,
                    detectionCount = detectedObjects.size,
                    rawX = rawX,
                    rawY = rawY,
                    filteredX = filteredX,
                    filteredY = filteredY,
                    velocityX = kalmanFilterX.velocity,
                    velocityY = kalmanFilterY.velocity,
                    dtSeconds = kalmanFilterX.lastDt
                )

                val result = MainActivity.DetectionResult(
                    objects = objectInfos,
                    imageWidth = frameWidth,
                    imageHeight = frameHeight,
                    isFrontCamera = isFrontCamera,
                    handLandmarks = handLandmarks,
                    handChecked = shouldDetectHands
                )
                onResult(update, result)
            }
            .addOnCompleteListener {
                imageProxy.close()
            }
    } else {
        imageProxy.close()
    }
}

@OptIn(ExperimentalGetImage::class)
private fun ImageProxy.toBitmapInternal(rotationDegrees: Int): Bitmap {
    val yBuffer = planes[0].buffer
    val uBuffer = planes[1].buffer
    val vBuffer = planes[2].buffer

    val ySize = yBuffer.remaining()
    val uSize = uBuffer.remaining()
    val vSize = vBuffer.remaining()

    val nv21 = ByteArray(ySize + uSize + vSize)

    yBuffer.get(nv21, 0, ySize)
    vBuffer.get(nv21, ySize, vSize)
    uBuffer.get(nv21, ySize + vSize, uSize)

    val yuvImage = android.graphics.YuvImage(nv21, android.graphics.ImageFormat.NV21, width, height, null)
    val out = java.io.ByteArrayOutputStream()
    yuvImage.compressToJpeg(android.graphics.Rect(0, 0, width, height), 100, out)
    val imageBytes = out.toByteArray()
    val bitmap = android.graphics.BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size)
    if (rotationDegrees == 0) return bitmap
    val matrix = android.graphics.Matrix().apply { postRotate(rotationDegrees.toFloat()) }
    return Bitmap.createBitmap(bitmap, 0, 0, bitmap.width, bitmap.height, matrix, true)
}

private fun takePhoto(context: android.content.Context, imageCapture: ImageCapture, executor: ExecutorService) {
    val name = SimpleDateFormat("yyyy-MM-dd-HH-mm-ss-SSS", Locale.US).format(System.currentTimeMillis())
    val contentValues = ContentValues().apply {
        put(MediaStore.MediaColumns.DISPLAY_NAME, name)
        put(MediaStore.MediaColumns.MIME_TYPE, "image/jpeg")
        if (Build.VERSION.SDK_INT > Build.VERSION_CODES.P) {
            put(MediaStore.Images.Media.RELATIVE_PATH, "Pictures/CamX")
        }
    }
    val outputOptions = ImageCapture.OutputFileOptions.Builder(context.contentResolver, MediaStore.Images.Media.EXTERNAL_CONTENT_URI, contentValues).build()
    imageCapture.takePicture(outputOptions, executor, object : ImageCapture.OnImageSavedCallback {
        override fun onImageSaved(output: ImageCapture.OutputFileResults) {
            (context as? MainActivity)?.runOnUiThread { Toast.makeText(context, "Photo saved!", Toast.LENGTH_SHORT).show() }
        }
        override fun onError(exc: ImageCaptureException) { Log.e("CamX", "Photo capture failed", exc) }
    })
}

private fun startVideoRecording(context: android.content.Context, videoCapture: VideoCapture<Recorder>, executor: ExecutorService): Recording {
    val name = SimpleDateFormat("yyyy-MM-dd-HH-mm-ss-SSS", Locale.US).format(System.currentTimeMillis())
    val contentValues = ContentValues().apply {
        put(MediaStore.MediaColumns.DISPLAY_NAME, name)
        put(MediaStore.MediaColumns.MIME_TYPE, "video/mp4")
        if (Build.VERSION.SDK_INT > Build.VERSION_CODES.P) {
            put(MediaStore.Video.Media.RELATIVE_PATH, "Movies/CamX")
        }
    }
    val mediaStoreOutputOptions = MediaStoreOutputOptions.Builder(context.contentResolver, MediaStore.Video.Media.EXTERNAL_CONTENT_URI).setContentValues(contentValues).build()
    val pendingRecording = videoCapture.output.prepareRecording(context, mediaStoreOutputOptions)
    if (ContextCompat.checkSelfPermission(context, Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED) {
        pendingRecording.withAudioEnabled()
    }
    return pendingRecording.start(executor) { recordEvent ->
        if (recordEvent is VideoRecordEvent.Finalize) {
            if (!recordEvent.hasError()) {
                (context as? MainActivity)?.runOnUiThread { Toast.makeText(context, "Video saved!", Toast.LENGTH_SHORT).show() }
            }
        }
    }
}
