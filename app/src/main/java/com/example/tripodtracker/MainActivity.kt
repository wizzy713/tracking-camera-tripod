package com.example.tripodtracker

import android.Manifest
import android.content.BroadcastReceiver
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ApplicationInfo
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
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
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
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.example.tripodtracker.ui.theme.TripodTrackerTheme
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetector as BallDetector
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarker
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
import kotlin.math.abs
import kotlin.math.hypot

// How far ahead the Kalman filter predicts, in seconds, to compensate for the
// loop delay. Tuned on the pendulum rig (2026-10-06): 0.20 s tracked as well as
// 0.25 s with fewer commands pointing the wrong way (8% against 9-11%); 0.35 s
// overshot the turning points (17%).
// Live-tunable from the Experiment tab (MainActivity.predictionHorizonSeconds);
// this is just the value it starts at.
private const val DEFAULT_PREDICTION_HORIZON_SECONDS = 0.2f

/**
 * Which pipeline is used to find the tracked subject. FACE (default) is the
 * existing ML Kit face-lock behavior. BALL finds an optic-yellow tennis ball by
 * colour (ColorBallDetector, robust to a net around it), falling back to
 * MediaPipe's Object Detector with an EfficientDet-Lite0 COCO model filtered to
 * the "sports ball" class, so only balls are tracked -- e.g. for the pendulum test in TESTING.md. BODY uses
 * the same COCO model filtered to the "person" class, so the box (and the aim
 * point) covers the whole body rather than just the face, and keeps tracking
 * when the subject turns away. Selectable from the Experiment tab or the
 * camera-screen toggle icon.
 */
enum class DetectionMode { FACE, BODY, BALL }

// Tripod address persistence (Settings screen -> SharedPreferences).
private const val PREFS_NAME = "tripod_connection"
private const val TUNE_PREFS = "tune_hook"
private const val PREF_ESP32_IP = "esp32_ip"
private const val PREF_UDP_PORT = "udp_port"
private const val DEFAULT_ESP32_IP = "10.47.140.33"
private const val DEFAULT_UDP_PORT = 4210
// Auto-discovery: probe every DISCOVERY_INTERVAL_MS once the tripod has been
// silent for DISCOVERY_SILENCE_MS (its telemetry normally arrives every ~2 s).
private const val DISCOVERY_INTERVAL_MS = 2000L
private const val DISCOVERY_SILENCE_MS = 5000L

// Advanced settings (Settings screen -> SharedPreferences).
private const val ADVANCED_PREFS = "advanced_settings"
private const val PREF_MANUAL_MODE = "manual_mode"
private const val PREF_AUTO_ZOOM = "auto_zoom"
private const val PREF_AUTO_ZOOM_TARGET = "auto_zoom_target"
private const val PREF_GESTURE_PALM = "gesture_open_palm"
private const val PREF_GESTURE_FIST = "gesture_fist"
private const val PREF_GESTURE_VICTORY = "gesture_victory"
// Per-mode tuning (see ModeProfile): one serialized profile per DetectionMode.
private const val PROFILE_PREFS = "mode_profiles"
// Manual joystick: how often the stick position is sent. The firmware stops the
// motors after 500 ms of silence, so this also keeps a held stick alive.
private const val JOYSTICK_SEND_INTERVAL_MS = 33L
// The smaller stick axis is ignored while it is below this fraction of the
// larger one (tan 30 deg) -- see snapToAxis.
private const val JOYSTICK_AXIS_SNAP = 0.58f
// Auto zoom: the subject size held (larger of its width and height, as a
// fraction of the frame) and the most zoom it may apply. Zooming in makes the
// same servo speed sweep the frame faster, and the servos' slowest speed is
// already coarse at 1x (see the dead-band note in camx_tripod.ino), so the cap
// is deliberately modest.
private const val DEFAULT_AUTO_ZOOM_TARGET = 0.35f
private const val AUTO_ZOOM_MAX_RATIO = 3f

// COCO model bundled in assets/ for DetectionMode.BALL, and the only class kept.
private const val BALL_MODEL_ASSET = "efficientdet_lite0.tflite"
private const val BALL_CATEGORY = "sports ball"
// Balls are small and often motion-blurred at the bottom of a swing, so this is
// lower than MediaPipe's usual 0.5 default; raise it if false positives appear.
private const val BALL_SCORE_THRESHOLD = 0.3f
// DetectionMode.BODY: same model, "person" class only. People are large and
// well-represented in COCO, so MediaPipe's usual 0.5 threshold is fine.
private const val PERSON_CATEGORY = "person"
private const val PERSON_SCORE_THRESHOLD = 0.5f
// BODY-mode aim point. Aiming at the person box's centre (the hips) pushes the
// head out of the top of the frame whenever the subject is close, so the aim
// is pulled up towards the face: FACE_AIM_OFFSET face-heights below the face
// centre (about the chest), never lower than the body centre. With no face
// visible (subject turned away) it falls back to this fraction of the box
// height from its top, which lands in about the same place on a full body.
private const val FACE_AIM_OFFSET = 1.5f
private const val BODY_AIM_FALLBACK_FRACTION = 0.25f

// DetectionMode.BODY's primary detector: MediaPipe Pose Landmarker (BlazePose),
// which finds people specifically and returns 33 body landmarks each. The COCO
// "person" detector above remains as the fallback for frames where it finds no
// one (it reaches further: BlazePose needs the person fairly large in frame).
private const val POSE_MODEL_ASSET = "pose_landmarker_lite.task"
private const val MAX_POSES = 3

// Hand gestures. The hand pass runs every Nth frame: often while looking for a
// subject to lock, less often once locked, when it is only watching for a
// gesture command and its inference time comes out of the tracking loop.
private const val HAND_CHECK_INTERVAL_UNLOCKED = 5
private const val HAND_CHECK_INTERVAL_LOCKED = 10
// Minimum time between two gesture-triggered record toggles.
private const val GESTURE_RECORD_COOLDOWN_MS = 3000L

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

/** Which hand gestures are acted on (Advanced Settings). All on by default. */
data class GestureSettings(
    val openPalm: Boolean = true,
    val fist: Boolean = true,
    val victory: Boolean = true
) {
    val any: Boolean get() = openPalm || fist || victory
    // Once a subject is locked the open palm has nothing left to do (it locks,
    // or resumes a pause, and a pause releases the lock).
    val anyWhileLocked: Boolean get() = fist || victory
}

class MainActivity : ComponentActivity() {

    private lateinit var cameraExecutor: ExecutorService
    private val kalmanFilterX = KalmanFilter()
    private val kalmanFilterY = KalmanFilter()
    private val udpSender = UdpSender()
    lateinit var logManager: LogManager
    private lateinit var imuReader: ImuReader
    private lateinit var roomFrame: RoomFrame
    @Volatile var handLandmarker: HandLandmarker? = null

    // Tripod address. Loaded from / saved to SharedPreferences (see onCreate and
    // the Settings screen's onConnect), so it survives app restarts; these are
    // only the first-run defaults.
    private var esp32Ip by mutableStateOf(DEFAULT_ESP32_IP)
    private var udpPort by mutableIntStateOf(DEFAULT_UDP_PORT)
    private var isLogging by mutableStateOf(false)
    private var currentScreen by mutableStateOf("camera")
    private var lockedId by mutableStateOf<Int?>(null)
    // Set by the closed-fist gesture: detection keeps running (so an open palm
    // can resume it) but the tripod is told to hold still.
    private var trackingPaused by mutableStateOf(false)
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
    // Hold-still band (see SettleHold), from the mode's profile. Exit 0 turns it off.
    private val settleHoldX = SettleHold()
    private val settleHoldY = SettleHold()
    @Volatile private var holdEnter = 0f
    @Volatile private var holdExit = 0f
    @Volatile private var holdSpeed = 0f

    private var testName by mutableStateOf("")
    private var testNotes by mutableStateOf("")
    private var loggingStartedUnsynced = false

    // Advanced settings, saved to SharedPreferences (see saveAdvancedSettings).
    // In manual mode the on-screen joystick drives the tripod (JOY packets) and
    // the tracking error is not sent.
    private var manualMode by mutableStateOf(false)
    private var autoZoomEnabled by mutableStateOf(false)
    private var autoZoomTarget by mutableFloatStateOf(DEFAULT_AUTO_ZOOM_TARGET)
    private var gestureSettings by mutableStateOf(GestureSettings())
    // The tuning profile of the current detection mode, as last applied or saved.
    private var modeProfile by mutableStateOf(ModeProfile.defaultFor(DetectionMode.FACE))

    data class DetectedObjectInfo(
        val boundingBox: Rect,
        val trackingId: Int?,
        val label: String = "Object",
        val isLocked: Boolean = false,
        // Point to centre in frame, if not the box centre (BODY mode).
        val aimX: Float? = null,
        val aimY: Float? = null,
        // Face matched to this body (BODY mode), drawn as a second box.
        val faceBox: Rect? = null,
        // Pose landmarks in frame pixels, null where not visible (BODY mode, when
        // the pose model found this person), drawn as a skeleton.
        val skeleton: List<Offset?>? = null
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
        val handChecked: Boolean = false,
        // What the hand is doing, on frames where handChecked is true.
        val gesture: HandGesture = HandGesture.NONE
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
        val measuredErrX: Float,
        val measuredErrY: Float,
        // The subject's own speed in half-frames per second, known only on an
        // axis tracked in room coordinates (see RoomFrame); 0 otherwise.
        val subjectSpeedX: Float,
        val subjectSpeedY: Float,
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
        imuReader = ImuReader(this).also { it.start() }
        roomFrame = RoomFrame(imuReader)
        if (applicationInfo.flags and ApplicationInfo.FLAG_DEBUGGABLE != 0) {
            ContextCompat.registerReceiver(
                this, tuneReceiver, IntentFilter("com.example.tripodtracker.TUNE"), ContextCompat.RECEIVER_EXPORTED
            )
            restoreTuneSettings()
        }

        // The gains reach the tripod once it is heard from (see CamXApp).
        applyModeProfile(detectionMode, sendToTripod = false)

        val prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
        esp32Ip = prefs.getString(PREF_ESP32_IP, null)?.takeIf { it.isNotBlank() } ?: DEFAULT_ESP32_IP
        udpPort = prefs.getInt(PREF_UDP_PORT, DEFAULT_UDP_PORT).takeIf { it in 1..65535 } ?: DEFAULT_UDP_PORT

        val advanced = getSharedPreferences(ADVANCED_PREFS, MODE_PRIVATE)
        manualMode = advanced.getBoolean(PREF_MANUAL_MODE, false)
        autoZoomEnabled = advanced.getBoolean(PREF_AUTO_ZOOM, false)
        autoZoomTarget = advanced.getFloat(PREF_AUTO_ZOOM_TARGET, DEFAULT_AUTO_ZOOM_TARGET)
        gestureSettings = GestureSettings(
            openPalm = advanced.getBoolean(PREF_GESTURE_PALM, true),
            fist = advanced.getBoolean(PREF_GESTURE_FIST, true),
            victory = advanced.getBoolean(PREF_GESTURE_VICTORY, true)
        )

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
        // When it is heard from again (first connection, or after it rebooted
        // and lost its runtime gains), it is sent the current mode's profile.
        LaunchedEffect(Unit) {
            var connected = false
            while (true) {
                delay(DISCOVERY_INTERVAL_MS)
                val heard = udpSender.lastRxFromTargetMs != 0L &&
                    SystemClock.elapsedRealtime() - udpSender.lastRxFromTargetMs <= DISCOVERY_SILENCE_MS
                if (!heard) {
                    udpSender.discover()
                } else if (!connected) {
                    udpSender.send(cfgPacket(modeProfile.toTripodConfig()))
                }
                connected = heard
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
            if (it) {
                // Ask for the gains if they were never synced (e.g. the app was
                // restarted); the reply lands well before the log is saved.
                loggingStartedUnsynced = tripodConfig == null
                if (loggingStartedUnsynced) udpSender.send("CFG?")
                logManager.startSession(buildSessionMetadata())
            } else {
                if (loggingStartedUnsynced) logManager.startSession(buildSessionMetadata())
                logManager.saveLog()
            }
        }

        when (currentScreen) {
            "settings" -> {
                BackHandler { currentScreen = "camera" }
                ConnectionScreen(
                    currentIp = esp32Ip,
                    currentPort = udpPort,
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    manualMode = manualMode,
                    onManualModeChange = { manualMode = it; saveAdvancedSettings() },
                    autoZoomEnabled = autoZoomEnabled,
                    onAutoZoomEnabledChange = { autoZoomEnabled = it; saveAdvancedSettings() },
                    autoZoomTarget = autoZoomTarget,
                    onAutoZoomTargetChange = { autoZoomTarget = it; saveAdvancedSettings() },
                    gestureSettings = gestureSettings,
                    onGestureSettingsChange = { gestureSettings = it; saveAdvancedSettings() },
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
                        // The tab edits the gains; the slew limit stays the profile's.
                        saveModeProfile(modeProfile.withTripodConfig(config))
                        udpSender.send(cfgPacket(modeProfile.toTripodConfig()))
                    },
                    modeProfile = modeProfile,
                    onFilterTuningChanged = { saveModeProfile(currentFilterTuning()) },
                    onResetModeProfile = {
                        getSharedPreferences(PROFILE_PREFS, MODE_PRIVATE).edit().remove(detectionMode.name).apply()
                        applyModeProfile(detectionMode, sendToTripod = true)
                    },
                    onSyncFromTripod = { udpSender.send("CFG?") },
                    kalmanFilterX = kalmanFilterX,
                    kalmanFilterY = kalmanFilterY,
                    predictionHorizonSeconds = predictionHorizonSeconds,
                    onPredictionHorizonChange = {
                        predictionHorizonSeconds = it
                        saveModeProfile(currentFilterTuning())
                    },
                    detectionMode = detectionMode,
                    onDetectionModeChange = { mode -> switchDetectionMode(mode) },
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
                    roomFrame,
                    handLandmarker,
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    onOpenSettings = { currentScreen = "settings" },
                    onOpenExperiment = { currentScreen = "experiment" },
                    lockedId = lockedId,
                    battery = batteryStatus,
                    detectionMode = detectionMode,
                    onDetectionModeChange = { mode -> switchDetectionMode(mode) },
                    predictionHorizonSeconds = predictionHorizonSeconds,
                    manualMode = manualMode,
                    trackingPaused = trackingPaused,
                    onTrackingPausedChange = { trackingPaused = it },
                    autoZoomEnabled = autoZoomEnabled,
                    autoZoomTarget = autoZoomTarget,
                    gestureSettings = gestureSettings,
                    onManualDrive = { x, y ->
                        udpSender.send(String.format(Locale.US, "JOY:%.3f,%.3f,SEQ:%d", x, y, packetSeq++))
                    },
                    onUnlock = { lockedId = null },
                    onTargetUpdate = { id -> lockedId = id }
                ) { update ->
                    val seq = packetSeq++
                    // In manual mode the joystick owns the servos; the error is
                    // still logged below, which records how the frame moves under
                    // a known stick command.
                    // Hold-still band: what is sent, and logged, is the gated error.
                    val speedLimit = holdSpeed
                    val sentErrX = settleHoldX.apply(
                        update.errX, update.measuredErrX, holdEnter, holdExit,
                        subjectMoving = speedLimit > 0f && update.subjectSpeedX > speedLimit
                    )
                    val sentErrY = settleHoldY.apply(
                        update.errY, update.measuredErrY, holdEnter, holdExit,
                        subjectMoving = speedLimit > 0f && update.subjectSpeedY > speedLimit
                    )
                    if (!manualMode) {
                        udpSender.send(String.format(Locale.US, "EX:%.4f,EY:%.4f,SEQ:%d", sentErrX, sentErrY, seq))
                    }
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
                            errX = sentErrX,
                            errY = sentErrY,
                            imu = imuReader.sampleFor(update.frameTimestampNanos)
                        )
                    }
                }
            }
        }

        val onToggleLogging: (Boolean) -> Unit = {
            isLogging = it
            if (it) {
                // Ask for the gains if they were never synced (e.g. the app was
                // restarted); the reply lands well before the log is saved.
                loggingStartedUnsynced = tripodConfig == null
                if (loggingStartedUnsynced) udpSender.send("CFG?")
                logManager.startSession(buildSessionMetadata())
            } else {
                if (loggingStartedUnsynced) logManager.startSession(buildSessionMetadata())
                logManager.saveLog()
            }
        }

        when (currentScreen) {
            "settings" -> {
                BackHandler { currentScreen = "camera" }
                ConnectionScreen(
                    currentIp = esp32Ip,
                    currentPort = udpPort,
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    manualMode = manualMode,
                    onManualModeChange = { manualMode = it; saveAdvancedSettings() },
                    autoZoomEnabled = autoZoomEnabled,
                    onAutoZoomEnabledChange = { autoZoomEnabled = it; saveAdvancedSettings() },
                    autoZoomTarget = autoZoomTarget,
                    onAutoZoomTargetChange = { autoZoomTarget = it; saveAdvancedSettings() },
                    gestureSettings = gestureSettings,
                    onGestureSettingsChange = { gestureSettings = it; saveAdvancedSettings() },
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
                        // The tab edits the gains; the slew limit stays the profile's.
                        saveModeProfile(modeProfile.withTripodConfig(config))
                        udpSender.send(cfgPacket(modeProfile.toTripodConfig()))
                    },
                    modeProfile = modeProfile,
                    onFilterTuningChanged = { saveModeProfile(currentFilterTuning()) },
                    onResetModeProfile = {
                        getSharedPreferences(PROFILE_PREFS, MODE_PRIVATE).edit().remove(detectionMode.name).apply()
                        applyModeProfile(detectionMode, sendToTripod = true)
                    },
                    onSyncFromTripod = { udpSender.send("CFG?") },
                    kalmanFilterX = kalmanFilterX,
                    kalmanFilterY = kalmanFilterY,
                    predictionHorizonSeconds = predictionHorizonSeconds,
                    onPredictionHorizonChange = {
                        predictionHorizonSeconds = it
                        saveModeProfile(currentFilterTuning())
                    },
                    detectionMode = detectionMode,
                    onDetectionModeChange = { mode -> switchDetectionMode(mode) },
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
                    roomFrame,
                    handLandmarker,
                    isLogging = isLogging,
                    onToggleLogging = onToggleLogging,
                    onOpenSettings = { currentScreen = "settings" },
                    onOpenExperiment = { currentScreen = "experiment" },
                    lockedId = lockedId,
                    battery = batteryStatus,
                    detectionMode = detectionMode,
                    onDetectionModeChange = { mode -> switchDetectionMode(mode) },
                    predictionHorizonSeconds = predictionHorizonSeconds,
                    manualMode = manualMode,
                    trackingPaused = trackingPaused,
                    onTrackingPausedChange = { trackingPaused = it },
                    autoZoomEnabled = autoZoomEnabled,
                    autoZoomTarget = autoZoomTarget,
                    gestureSettings = gestureSettings,
                    onManualDrive = { x, y ->
                        udpSender.send(String.format(Locale.US, "JOY:%.3f,%.3f,SEQ:%d", x, y, packetSeq++))
                    },
                    onUnlock = { lockedId = null },
                    onTargetUpdate = { id -> lockedId = id }
                ) { update ->
                    val seq = packetSeq++
                    // In manual mode the joystick owns the servos; the error is
                    // still logged below, which records how the frame moves under
                    // a known stick command.
                    // Hold-still band: what is sent, and logged, is the gated error.
                    val speedLimit = holdSpeed
                    val sentErrX = settleHoldX.apply(
                        update.errX, update.measuredErrX, holdEnter, holdExit,
                        subjectMoving = speedLimit > 0f && update.subjectSpeedX > speedLimit
                    )
                    val sentErrY = settleHoldY.apply(
                        update.errY, update.measuredErrY, holdEnter, holdExit,
                        subjectMoving = speedLimit > 0f && update.subjectSpeedY > speedLimit
                    )
                    if (!manualMode) {
                        udpSender.send(String.format(Locale.US, "EX:%.4f,EY:%.4f,SEQ:%d", sentErrX, sentErrY, seq))
                    }
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
                            errX = sentErrX,
                            errY = sentErrY,
                            imu = imuReader.sampleFor(update.frameTimestampNanos)
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
            "hold_enter" to holdEnter.toString(),
            "hold_exit" to holdExit.toString(),
            "hold_speed" to holdSpeed.toString(),
            "gyro_available" to imuReader.available.toString(),
            "room_frame" to if (roomFrame.active) roomFrame.mode.toString() else "0",
            "delay_compensation" to roomFrame.delayCompensation.toString(),
            "fov_half_angle_x_deg" to roomFrame.halfAngleXDeg.toString(),
            "fov_half_angle_x_rear_deg" to roomFrame.halfAngleRearXDeg.toString(),
            "tripod_kp" to (cfg?.kp?.toString() ?: "unknown (not synced)"),
            "tripod_ki" to (cfg?.ki?.toString() ?: "unknown (not synced)"),
            "tripod_kd" to (cfg?.kd?.toString() ?: "unknown (not synced)"),
            "tripod_max_speed_offset_us" to (cfg?.maxSpeedOffsetUs?.toString() ?: "unknown (not synced)"),
            "tripod_deadzone" to (cfg?.deadzone?.toString() ?: "unknown (not synced)"),
            "tripod_min_offset_us" to (cfg?.minOffsetUs?.toString() ?: "unknown (not synced)"),
            "tripod_slew_us_per_s" to (cfg?.slewUsPerS?.toString() ?: "unknown (not synced)"),
            "manual_mode" to manualMode.toString(),
            "auto_zoom" to if (autoZoomEnabled) autoZoomTarget.toString() else "off"
        )
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

    private fun cfgPacket(config: TripodConfig): String = String.format(
        Locale.US,
        "CFG:KP:%.2f,KI:%.2f,KD:%.2f,MS:%.1f,DZ:%.3f,MO:%.1f,SL:%.0f",
        config.kp, config.ki, config.kd, config.maxSpeedOffsetUs, config.deadzone, config.minOffsetUs, config.slewUsPerS
    )

    private fun switchDetectionMode(mode: DetectionMode) {
        detectionMode = mode
        lockedId = null
        kalmanFilterX.reset()
        kalmanFilterY.reset()
        applyModeProfile(mode, sendToTripod = true)
    }

    /** Loads [mode]'s saved tuning (or its defaults) into the filters, the horizon and, optionally, the tripod. */
    private fun applyModeProfile(mode: DetectionMode, sendToTripod: Boolean) {
        val profile = ModeProfile.parse(
            getSharedPreferences(PROFILE_PREFS, MODE_PRIVATE).getString(mode.name, null),
            ModeProfile.defaultFor(mode)
        )
        modeProfile = profile
        kalmanFilterX.measurementNoise = profile.measurementNoise
        kalmanFilterY.measurementNoise = profile.measurementNoise
        kalmanFilterX.accelerationNoise = profile.accelerationNoise
        kalmanFilterY.accelerationNoise = profile.accelerationNoise
        predictionHorizonSeconds = profile.predictionHorizonSeconds
        holdEnter = profile.holdEnter
        holdExit = profile.holdExit
        holdSpeed = profile.holdSpeed
        if (sendToTripod) udpSender.send(cfgPacket(profile.toTripodConfig()))
    }

    /** The current mode's profile with the filter settings now in effect. */
    private fun currentFilterTuning() = modeProfile.copy(
        measurementNoise = kalmanFilterX.measurementNoise,
        accelerationNoise = kalmanFilterX.accelerationNoise,
        predictionHorizonSeconds = predictionHorizonSeconds
    )

    private fun saveModeProfile(profile: ModeProfile) {
        modeProfile = profile
        getSharedPreferences(PROFILE_PREFS, MODE_PRIVATE).edit()
            .putString(detectionMode.name, profile.serialize())
            .apply()
    }

    private fun saveAdvancedSettings() {
        getSharedPreferences(ADVANCED_PREFS, MODE_PRIVATE).edit()
            .putBoolean(PREF_MANUAL_MODE, manualMode)
            .putBoolean(PREF_AUTO_ZOOM, autoZoomEnabled)
            .putFloat(PREF_AUTO_ZOOM_TARGET, autoZoomTarget)
            .putBoolean(PREF_GESTURE_PALM, gestureSettings.openPalm)
            .putBoolean(PREF_GESTURE_FIST, gestureSettings.fist)
            .putBoolean(PREF_GESTURE_VICTORY, gestureSettings.victory)
            .apply()
    }

    // Debug-build tuning hook: lets a computer set up the next test run over adb
    // instead of the Experiment-tab sliders, e.g.
    //   adb shell am broadcast -a com.example.tripodtracker.TUNE \
    //       --ef kp 110 --ef ki 0 --ef kd 0 --ef h 0 --ef r 100 --ef sa 150 --es name R3
    // Every extra is optional; PID keys left out keep the tripod's current value.
    private val tuneReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            fun f(key: String) = if (intent.hasExtra(key)) intent.getFloatExtra(key, 0f) else null
            // Kept across an app restart (the phone is unplugged and carried to
            // the rig between "apply" and "record"), see restoreTuneSettings().
            val edit = getSharedPreferences(TUNE_PREFS, MODE_PRIVATE).edit()
            for (key in listOf("r", "sa", "h", "rc", "dc", "fov", "fovr")) f(key)?.let { edit.putFloat(key, it) }
            // The hold-still band used to be kept here; it now lives in the mode's profile.
            edit.remove("he").remove("hx")
            for (key in listOf("name", "notes", "mode")) intent.getStringExtra(key)?.let { edit.putString(key, it) }
            edit.apply()
            restoreTuneSettings()
            val pid = listOf("kp" to "KP", "ki" to "KI", "kd" to "KD", "ms" to "MS", "dz" to "DZ", "mo" to "MO", "td" to "TD", "sl" to "SL")
                .mapNotNull { (extra, key) -> f(extra)?.let { String.format(Locale.US, "%s:%.3f", key, it) } }
            // The tripod answers every CFG packet with its (clamped) gains, which
            // updates tripodConfig and so the log header.
            udpSender.send(if (pid.isEmpty()) "CFG?" else "CFG:" + pid.joinToString(","))
            // Keep what this run set as the mode's saved tuning too.
            saveModeProfile(currentFilterTuning().let { p ->
                p.copy(
                    kp = f("kp") ?: p.kp, ki = f("ki") ?: p.ki, kd = f("kd") ?: p.kd,
                    maxSpeedOffsetUs = f("ms") ?: p.maxSpeedOffsetUs,
                    deadzone = f("dz") ?: p.deadzone, minOffsetUs = f("mo") ?: p.minOffsetUs,
                    holdEnter = f("he") ?: p.holdEnter, holdExit = f("hx") ?: p.holdExit,
                    holdSpeed = f("hv") ?: p.holdSpeed, slewUsPerS = f("sl") ?: p.slewUsPerS
                )
            })
            holdEnter = modeProfile.holdEnter
            holdExit = modeProfile.holdExit
            holdSpeed = modeProfile.holdSpeed
            Toast.makeText(context, "Run $testName ready: " + pid.joinToString(" ") +
                " H=$predictionHorizonSeconds R=${kalmanFilterX.measurementNoise} sa=${kalmanFilterX.accelerationNoise} hold=$holdEnter/$holdExit",
                Toast.LENGTH_LONG).show()
            Log.i("TuneHook", "applied h=$predictionHorizonSeconds R=${kalmanFilterX.measurementNoise} " +
                "sa=${kalmanFilterX.accelerationNoise} hold=$holdEnter/$holdExit room=${roomFrame.mode} dc=${roomFrame.delayCompensation} name=$testName pid=$pid")
        }
    }

    private fun restoreTuneSettings() {
        val prefs = getSharedPreferences(TUNE_PREFS, MODE_PRIVATE)
        // Mode first: selecting it loads that mode's saved profile, which the
        // hook's own r/sa/h values below then override.
        prefs.getString("mode", null)?.let { name ->
            DetectionMode.entries.firstOrNull { it.name == name }?.takeIf { it != detectionMode }?.let {
                detectionMode = it
                kalmanFilterX.reset()
                kalmanFilterY.reset()
                applyModeProfile(it, sendToTripod = false)
            }
        }
        if (prefs.contains("r")) prefs.getFloat("r", 0f).let { kalmanFilterX.measurementNoise = it; kalmanFilterY.measurementNoise = it }
        if (prefs.contains("sa")) prefs.getFloat("sa", 0f).let { kalmanFilterX.accelerationNoise = it; kalmanFilterY.accelerationNoise = it }
        if (prefs.contains("h")) predictionHorizonSeconds = prefs.getFloat("h", 0f)
        // Room-coordinate tracking (see RoomFrame): "rc" 0 off, 1 pan, 2 pan and
        // tilt; "dc" 1 for delay compensation; "fov"/"fovr" the horizontal
        // half-angle with the front/rear camera.
        if (prefs.contains("fov")) roomFrame.halfAngleXDeg = prefs.getFloat("fov", RoomFrame.DEFAULT_HALF_ANGLE_X_DEG)
        if (prefs.contains("fovr")) roomFrame.halfAngleRearXDeg = prefs.getFloat("fovr", RoomFrame.DEFAULT_HALF_ANGLE_X_DEG)
        if (prefs.contains("dc")) roomFrame.delayCompensation = prefs.getFloat("dc", 0f) >= 0.5f
        if (prefs.contains("rc")) {
            val mode = prefs.getFloat("rc", 0f).toInt().coerceIn(RoomFrame.OFF, RoomFrame.PAN_TILT)
            if (mode != roomFrame.mode) {
                // The filters change coordinates with the mode, so start them afresh.
                roomFrame.mode = mode
                kalmanFilterX.reset()
                kalmanFilterY.reset()
            }
        }
        prefs.getString("name", null)?.let { testName = it }
        prefs.getString("notes", null)?.let { testNotes = it }
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
            "hold_enter" to holdEnter.toString(),
            "hold_exit" to holdExit.toString(),
            "hold_speed" to holdSpeed.toString(),
            "gyro_available" to imuReader.available.toString(),
            "room_frame" to if (roomFrame.active) roomFrame.mode.toString() else "0",
            "delay_compensation" to roomFrame.delayCompensation.toString(),
            "fov_half_angle_x_deg" to roomFrame.halfAngleXDeg.toString(),
            "fov_half_angle_x_rear_deg" to roomFrame.halfAngleRearXDeg.toString(),
            "tripod_kp" to (cfg?.kp?.toString() ?: "unknown (not synced)"),
            "tripod_ki" to (cfg?.ki?.toString() ?: "unknown (not synced)"),
            "tripod_kd" to (cfg?.kd?.toString() ?: "unknown (not synced)"),
            "tripod_max_speed_offset_us" to (cfg?.maxSpeedOffsetUs?.toString() ?: "unknown (not synced)"),
            "tripod_deadzone" to (cfg?.deadzone?.toString() ?: "unknown (not synced)"),
            "tripod_min_offset_us" to (cfg?.minOffsetUs?.toString() ?: "unknown (not synced)"),
            "tripod_slew_us_per_s" to (cfg?.slewUsPerS?.toString() ?: "unknown (not synced)"),
            "manual_mode" to manualMode.toString(),
            "auto_zoom" to if (autoZoomEnabled) autoZoomTarget.toString() else "off"
        )
    }

    override fun onDestroy() {
        super.onDestroy()
        imuReader.stop()
        cameraExecutor.shutdown()
        try {
            cameraExecutor.awaitTermination(2, TimeUnit.SECONDS)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        try { unregisterReceiver(tuneReceiver) } catch (e: IllegalArgumentException) { /* release build: never registered */ }
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
    roomFrame: RoomFrame,
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
    manualMode: Boolean,
    trackingPaused: Boolean,
    onTrackingPausedChange: (Boolean) -> Unit,
    autoZoomEnabled: Boolean,
    autoZoomTarget: Float,
    gestureSettings: GestureSettings,
    onManualDrive: (Float, Float) -> Unit,
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
    val currentTrackingPaused by rememberUpdatedState(trackingPaused)
    val currentAutoZoomEnabled by rememberUpdatedState(autoZoomEnabled)
    val currentAutoZoomTarget by rememberUpdatedState(autoZoomTarget)
    val currentGestures by rememberUpdatedState(gestureSettings)
    val currentOnManualDrive by rememberUpdatedState(onManualDrive)

    var cameraSelector by remember { mutableStateOf(CameraSelector.DEFAULT_BACK_CAMERA) }
    var detectionResult by remember { mutableStateOf<MainActivity.DetectionResult?>(null) }
    var handOverlay by remember { mutableStateOf<MainActivity.HandOverlay?>(null) }
    var isTrackingEnabled by remember { mutableStateOf(true) }
    // Joystick deflection in screen coordinates, each axis in [-1, 1] (right and
    // down positive); zero when released.
    var joystick by remember { mutableStateOf(Offset.Zero) }
    var flashMode by remember { mutableIntStateOf(ImageCapture.FLASH_MODE_OFF) }

    val imageCapture = remember { ImageCapture.Builder().setFlashMode(flashMode).build() }
    val recorder = remember { Recorder.Builder().setQualitySelector(QualitySelector.from(Quality.HIGHEST)).build() }
    val videoCapture = remember { VideoCapture.withOutput(recorder) }
    var activeRecording by remember { mutableStateOf<Recording?>(null) }

    val cameraProviderFuture = remember { ProcessCameraProvider.getInstance(context) }

    Box(modifier = Modifier.fillMaxSize()) {
        AndroidView(factory = { previewView }, modifier = Modifier.fillMaxSize())

        if (isTrackingEnabled || manualMode) {
            Text(
                text = when {
                    manualMode -> "MANUAL"
                    trackingPaused -> if (gestureSettings.openPalm) "PAUSED\nopen palm to resume" else "PAUSED\ntap the tracking icon to resume"
                    lockedId != null -> "LOCKED"
                    detectionResult != null -> "Tracking"
                    else -> "Searching"
                },
                color = if (manualMode || trackingPaused) Color.Yellow else if (lockedId != null) Color.Cyan else Color.Green,
                textAlign = TextAlign.Center,
                modifier = Modifier.padding(top = 64.dp).align(Alignment.TopCenter),
                style = MaterialTheme.typography.headlineSmall
            )
        }

        if (isTrackingEnabled) {
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

                        // BODY mode: the face the aim point is anchored to.
                        obj.faceBox?.let { face ->
                            val fLeft = if (isFront) size.width - (face.right * scaleX) else face.left * scaleX
                            val fRight = if (isFront) size.width - (face.left * scaleX) else face.right * scaleX
                            drawRect(
                                color = Color.Yellow,
                                topLeft = Offset(fLeft, face.top * scaleY),
                                size = Size(fRight - fLeft, (face.bottom - face.top) * scaleY),
                                style = Stroke(width = 3.dp.toPx())
                            )
                        }

                        // BODY mode: the pose model's skeleton for this person.
                        obj.skeleton?.let { joints ->
                            val pts = joints.map { j ->
                                j?.let { Offset(if (isFront) size.width - it.x * scaleX else it.x * scaleX, it.y * scaleY) }
                            }
                            POSE_CONNECTIONS.forEach { (a, b) ->
                                val start = pts.getOrNull(a)
                                val end = pts.getOrNull(b)
                                if (start != null && end != null) {
                                    drawLine(color = Color.Yellow, start = start, end = end, strokeWidth = 3.dp.toPx())
                                }
                            }
                        }
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
                    val next = when (detectionMode) {
                        DetectionMode.FACE -> DetectionMode.BODY
                        DetectionMode.BODY -> DetectionMode.BALL
                        DetectionMode.BALL -> DetectionMode.FACE
                    }
                    onDetectionModeChange(next)
                }) {
                    Icon(
                        imageVector = when (detectionMode) {
                            DetectionMode.FACE -> Icons.Default.Face
                            DetectionMode.BODY -> Icons.Default.Accessibility
                            DetectionMode.BALL -> Icons.Default.SportsBaseball
                        },
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
                    // A fist-paused tracker is resumed, not switched off, by a tap.
                    if (trackingPaused) {
                        onTrackingPausedChange(false)
                        return@IconButton
                    }
                    isTrackingEnabled = !isTrackingEnabled
                    if (!isTrackingEnabled) {
                        onUnlock()
                        kalmanFilterX.reset()
                        kalmanFilterY.reset()
                        onTargetDetected(
                            MainActivity.TrackingUpdate(
                                errX = 0f, errY = 0f, measuredErrX = 0f, measuredErrY = 0f, subjectSpeedX = 0f, subjectSpeedY = 0f,
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
                        tint = if (!isTrackingEnabled) Color.White else if (trackingPaused) Color.Yellow else Color.Green
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

        if (manualMode) {
            Joystick(
                modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 160.dp),
                onMove = { joystick = it }
            )

            // Stick to tripod signs, set on the rig and the same for both cameras:
            // both axes are negated relative to the tracking-error convention the
            // JOY packet uses (stick right or up sends what a subject left of or
            // below centre would).
            LaunchedEffect(Unit) {
                while (true) {
                    val command = snapToAxis(joystick)
                    currentOnManualDrive(-command.x, -command.y)
                    delay(JOYSTICK_SEND_INTERVAL_MS)
                }
            }
            // Leaving manual mode (or this screen) with the stick held: stop now
            // rather than after the firmware's loss-of-signal timeout.
            DisposableEffect(Unit) {
                onDispose {
                    joystick = Offset.Zero
                    currentOnManualDrive(0f, 0f)
                }
            }
        }

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

            // COCO detectors for DetectionMode.BALL ("sports ball") and BODY
            // ("person"). Each is created lazily on the analysis executor the first
            // time its mode sees a frame (model load is too slow for the main
            // thread, and FACE-only sessions never pay for it), and only ever used
            // and closed on that same thread.
            var ballDetector: BallDetector? = null
            var personDetector: BallDetector? = null
            var poseLandmarker: PoseLandmarker? = null
            var poseLandmarkerFailed = false
            // Set (on the executor) once this effect is torn down. A frame already
            // queued for this effect's analyzer can still run after the close task,
            // and calling detect() on a closed MediaPipe detector is a native
            // SIGSEGV, not a catchable exception -- so never hand one out.
            var detectorsClosed = false
            val ballTracker = BallTracker()
            // Separate instance so switching modes never carries IDs across.
            val personTracker = BallTracker()
            val colorBallDetector = ColorBallDetector()
            fun createCocoDetector(category: String, scoreThreshold: Float): BallDetector? = try {
                val options = BallDetector.ObjectDetectorOptions.builder()
                    .setBaseOptions(BaseOptions.builder().setModelAssetPath(BALL_MODEL_ASSET).build())
                    .setRunningMode(RunningMode.IMAGE)
                    .setCategoryAllowlist(listOf(category))
                    .setScoreThreshold(scoreThreshold)
                    .setMaxResults(5)
                    .build()
                BallDetector.createFromOptions(context, options)
            } catch (e: Exception) {
                Log.e("CamX", "COCO detector ($category) init failed: ${e.message}")
                null
            }
            val getBallDetector: () -> BallDetector? = {
                if (ballDetector == null && !detectorsClosed) {
                    ballDetector = createCocoDetector(BALL_CATEGORY, BALL_SCORE_THRESHOLD)
                }
                ballDetector
            }
            val getPersonDetector: () -> BallDetector? = {
                if (personDetector == null && !detectorsClosed) {
                    personDetector = createCocoDetector(PERSON_CATEGORY, PERSON_SCORE_THRESHOLD)
                }
                personDetector
            }

            val getPoseLandmarker: () -> PoseLandmarker? = {
                if (poseLandmarker == null && !poseLandmarkerFailed && !detectorsClosed) {
                    try {
                        val options = PoseLandmarker.PoseLandmarkerOptions.builder()
                            .setBaseOptions(BaseOptions.builder().setModelAssetPath(POSE_MODEL_ASSET).build())
                            .setRunningMode(RunningMode.IMAGE)
                            .setNumPoses(MAX_POSES)
                            .build()
                        poseLandmarker = PoseLandmarker.createFromOptions(context, options)
                    } catch (e: Exception) {
                        // Not retried every frame; BODY mode runs on the COCO
                        // person detector alone for this camera session.
                        poseLandmarkerFailed = true
                        Log.e("CamX", "Pose landmarker init failed: ${e.message}")
                    }
                }
                poseLandmarker
            }

            var frameCounter = 0
            var lostFrameCount = 0
            val gestureTrigger = GestureTrigger()
            var lastGestureRecordMs = 0L

            // Auto zoom state. All of it lives and dies with this camera binding:
            // CameraX resets the zoom to 1x whenever the camera is rebound.
            var camera: Camera? = null
            val autoZoom = AutoZoomController()
            var zoomRatio = 1f
            var lastZoomFrameNanos = 0L
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
                            getPersonDetector,
                            getPoseLandmarker,
                            colorBallDetector,
                            ballTracker,
                            personTracker,
                            currentDetectionMode,
                            currentLandmarker,
                            imageProxy,
                            cameraSelector == CameraSelector.DEFAULT_FRONT_CAMERA,
                            kalmanFilterX,
                            kalmanFilterY,
                            roomFrame,
                            currentPredictionHorizon,
                            currentLockedId,
                            lostFrameCount,
                            currentTrackingPaused,
                            // The hand pass (bitmap conversion + inference) takes
                            // executor time from the tracking pipeline, so it runs
                            // on a fraction of the frames -- fewer once locked,
                            // when it only has to catch a gesture command -- and
                            // not at all when no enabled gesture could act.
                            currentGestures.openPalm,
                            if (currentLockedId == null) {
                                currentGestures.any && frameCounter % HAND_CHECK_INTERVAL_UNLOCKED == 0
                            } else {
                                currentGestures.anyWhileLocked && frameCounter % HAND_CHECK_INTERVAL_LOCKED == 0
                            },
                            onTargetUpdate,
                            onLostFrameCountChanged = { lostFrameCount = it }
                        ) { update, result ->
                            onTargetDetected(update)

                            val zoomState = camera?.cameraInfo?.zoomState?.value
                            if (zoomState != null) {
                                val widest = maxOf(1f, zoomState.minZoomRatio)
                                val wanted = if (currentAutoZoomEnabled) {
                                    val dt = if (lastZoomFrameNanos == 0L) 0f else (update.frameTimestampNanos - lastZoomFrameNanos) / 1e9f
                                    // result.objects holds only the target (or nothing).
                                    val box = result.objects.firstOrNull()?.boundingBox
                                    val halfW = result.imageWidth / 2f
                                    val halfH = result.imageHeight / 2f
                                    autoZoom.update(
                                        subjectFraction = box?.let { maxOf(it.width() / (2f * halfW), it.height() / (2f * halfH)) },
                                        edgeExtent = box?.let {
                                            maxOf(
                                                abs(it.left - halfW) / halfW, abs(it.right - halfW) / halfW,
                                                abs(it.top - halfH) / halfH, abs(it.bottom - halfH) / halfH
                                            )
                                        } ?: 0f,
                                        targetFraction = currentAutoZoomTarget,
                                        currentRatio = zoomRatio,
                                        minRatio = widest,
                                        maxRatio = minOf(AUTO_ZOOM_MAX_RATIO, zoomState.maxZoomRatio),
                                        dtSeconds = dt
                                    )
                                } else {
                                    widest // switched off: back to the normal view
                                }
                                lastZoomFrameNanos = update.frameTimestampNanos
                                if (abs(wanted - zoomRatio) > 0.002f) {
                                    zoomRatio = wanted
                                    camera?.cameraControl?.setZoomRatio(wanted)
                                }
                            }
                            detectionResult = if (result.objects.isEmpty()) null else result
                            if (result.handChecked) {
                                // Gesture commands. (The open palm also picks the
                                // subject to lock, in processImageProxy.)
                                when (gestureTrigger.update(result.gesture)) {
                                    HandGesture.FIST -> if (currentGestures.fist && !currentTrackingPaused) {
                                        onUnlock()
                                        onTrackingPausedChange(true)
                                    }
                                    HandGesture.OPEN_PALM -> if (currentGestures.openPalm) onTrackingPausedChange(false)
                                    HandGesture.VICTORY -> if (currentGestures.victory) {
                                        val now = SystemClock.elapsedRealtime()
                                        if (now - lastGestureRecordMs > GESTURE_RECORD_COOLDOWN_MS) {
                                            lastGestureRecordMs = now
                                            val recording = activeRecording
                                            if (recording != null) {
                                                recording.stop()
                                                activeRecording = null
                                            } else {
                                                try {
                                                    activeRecording = startVideoRecording(context, videoCapture, executor)
                                                } catch (e: Exception) {
                                                    Log.e("CamX", "Gesture recording start failed: ${e.message}")
                                                }
                                            }
                                            Toast.makeText(
                                                context,
                                                if (activeRecording != null) "Recording started" else "Recording stopped",
                                                Toast.LENGTH_SHORT
                                            ).show()
                                        }
                                    }
                                    else -> {}
                                }

                                handOverlay = if (result.handLandmarks.size >= 21) {
                                    MainActivity.HandOverlay(
                                        landmarks = result.handLandmarks,
                                        isFrontCamera = result.isFrontCamera
                                    )
                                } else null
                            } else if (!currentGestures.any) {
                                handOverlay = null // hands are no longer checked
                            }
                        }
                    } else {
                        imageProxy.close()
                    }
                }
            }

            try {
                cameraProvider.unbindAll()
                camera = cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview, imageAnalyzer, imageCapture, videoCapture)
            } catch (exc: Exception) {
                Log.e("CameraX", "Full use-case binding failed, retrying without video capture", exc)
                try {
                    cameraProvider.unbindAll()
                    camera = cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview, imageAnalyzer, imageCapture)
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
                    detectorsClosed = true
                    ballDetector?.close()
                    ballDetector = null
                    personDetector?.close()
                    personDetector = null
                    poseLandmarker?.close()
                    poseLandmarker = null
                }
            }
        }
    }
}

/**
 * Keeps a mostly-sideways push from also tilting (and a mostly-vertical one from
 * panning): the smaller axis is dropped unless the stick is within about 30
 * degrees of a diagonal. Needed because the tripod's slowest speed is not slow
 * (the servo dead-band jump), so even a slight off-axis deflection moves the
 * other axis visibly.
 */
private fun snapToAxis(stick: Offset): Offset {
    val ax = abs(stick.x)
    val ay = abs(stick.y)
    return when {
        ay < JOYSTICK_AXIS_SNAP * ax -> Offset(stick.x, 0f)
        ax < JOYSTICK_AXIS_SNAP * ay -> Offset(0f, stick.y)
        else -> stick
    }
}

/**
 * On-screen joystick for manual mode. Reports the thumb position through
 * [onMove] with each axis in [-1, 1] (right and down positive), and springs back
 * to zero when the finger lifts.
 */
@Composable
fun Joystick(modifier: Modifier = Modifier, onMove: (Offset) -> Unit) {
    var thumb by remember { mutableStateOf(Offset.Zero) }
    val currentOnMove by rememberUpdatedState(onMove)

    Canvas(
        modifier = modifier.size(150.dp).pointerInput(Unit) {
            awaitEachGesture {
                // The thumb travels 60% of the pad radius, so full deflection is
                // reached before the finger leaves the pad.
                val centre = Offset(size.width / 2f, size.height / 2f)
                val travel = size.width / 2f * 0.6f
                fun moveTo(position: Offset) {
                    var v = (position - centre) / travel
                    val length = v.getDistance()
                    if (length > 1f) v /= length
                    thumb = v
                    currentOnMove(v)
                }

                val down = awaitFirstDown()
                moveTo(down.position)
                down.consume()
                while (true) {
                    val change = awaitPointerEvent().changes.firstOrNull { it.id == down.id }
                    if (change == null || !change.pressed) break
                    moveTo(change.position)
                    change.consume()
                }
                thumb = Offset.Zero
                currentOnMove(Offset.Zero)
            }
        }
    ) {
        val radius = size.minDimension / 2f
        drawCircle(color = Color.Black.copy(alpha = 0.35f), radius = radius)
        drawCircle(color = Color.White.copy(alpha = 0.7f), radius = radius, style = Stroke(width = 2.dp.toPx()))
        drawCircle(color = Color.White.copy(alpha = 0.85f), radius = radius * 0.4f, center = center + thumb * (radius * 0.6f))
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
    manualMode: Boolean,
    onManualModeChange: (Boolean) -> Unit,
    autoZoomEnabled: Boolean,
    onAutoZoomEnabledChange: (Boolean) -> Unit,
    autoZoomTarget: Float,
    onAutoZoomTargetChange: (Float) -> Unit,
    gestureSettings: GestureSettings,
    onGestureSettingsChange: (GestureSettings) -> Unit,
    onConnect: (String, Int) -> Unit,
    onTest: (String, Int, String) -> Unit
) {
    var ip by remember { mutableStateOf(currentIp) }
    var port by remember { mutableStateOf(currentPort.toString()) }
    var testMessage by remember { mutableStateOf("PING") }

    Column(
        modifier = Modifier.fillMaxSize().background(MaterialTheme.colorScheme.background)
            .verticalScroll(rememberScrollState()).padding(16.dp),
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

        Spacer(modifier = Modifier.height(16.dp))

        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)
        ) {
            Column(modifier = Modifier.padding(16.dp).fillMaxWidth()) {
                Text("Advanced Settings", style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.onSurface)
                Spacer(modifier = Modifier.height(8.dp))

                AdvancedSwitchRow(
                    title = "Manual joystick",
                    description = "Aim the tripod with an on-screen joystick. Tracking does not drive the tripod while this is on.",
                    checked = manualMode,
                    onCheckedChange = onManualModeChange
                )
                AdvancedSwitchRow(
                    title = "Auto zoom",
                    description = "Zoom in and out to keep the subject the same size in the frame.",
                    checked = autoZoomEnabled,
                    onCheckedChange = onAutoZoomEnabledChange
                )
                if (autoZoomEnabled) {
                    Text(
                        "Subject size: ${(autoZoomTarget * 100).toInt()}% of the frame",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.onSurface
                    )
                    Slider(value = autoZoomTarget, onValueChange = onAutoZoomTargetChange, valueRange = 0.1f..0.8f)
                }

                HorizontalDivider(modifier = Modifier.padding(vertical = 8.dp))
                Text("Hand gestures", style = MaterialTheme.typography.titleSmall, color = MaterialTheme.colorScheme.onSurface)
                Text(
                    "Raise the hand, fingers up, where the camera can see it.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
                AdvancedSwitchRow(
                    title = "Open palm",
                    description = "Lock onto the subject nearest the hand, and resume tracking after a fist.",
                    checked = gestureSettings.openPalm,
                    onCheckedChange = { onGestureSettingsChange(gestureSettings.copy(openPalm = it)) }
                )
                AdvancedSwitchRow(
                    title = "Closed fist",
                    description = "Stop auto tracking: the lock is released and the tripod holds still.",
                    checked = gestureSettings.fist,
                    onCheckedChange = { onGestureSettingsChange(gestureSettings.copy(fist = it)) }
                )
                AdvancedSwitchRow(
                    title = "Victory sign",
                    description = "Start or stop video recording.",
                    checked = gestureSettings.victory,
                    onCheckedChange = { onGestureSettingsChange(gestureSettings.copy(victory = it)) }
                )
            }
        }
    }
}

@Composable
private fun AdvancedSwitchRow(title: String, description: String, checked: Boolean, onCheckedChange: (Boolean) -> Unit) {
    Row(modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp), verticalAlignment = Alignment.CenterVertically) {
        Column(modifier = Modifier.weight(1f).padding(end = 12.dp)) {
            Text(title, style = MaterialTheme.typography.bodyLarge, color = MaterialTheme.colorScheme.onSurface)
            Text(description, style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
        }
        Switch(checked = checked, onCheckedChange = onCheckedChange)
    }
}

@OptIn(ExperimentalGetImage::class)
private fun processImageProxy(
    faceDetector: FaceDetector,
    getBallDetector: () -> BallDetector?,
    getPersonDetector: () -> BallDetector?,
    getPoseLandmarker: () -> PoseLandmarker?,
    colorBallDetector: ColorBallDetector,
    ballTracker: BallTracker,
    personTracker: BallTracker,
    mode: DetectionMode,
    handLandmarker: HandLandmarker?,
    imageProxy: ImageProxy,
    isFrontCamera: Boolean,
    kalmanFilterX: KalmanFilter,
    kalmanFilterY: KalmanFilter,
    roomFrame: RoomFrame,
    predictionHorizonSeconds: Float,
    lockedId: Int?,
    lostFrameCount: Int,
    paused: Boolean,
    palmLockEnabled: Boolean,
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
        var gesture = HandGesture.NONE

        // Upright RGB copy of the frame for the MediaPipe models (hand landmarker
        // and ball detector). Lazy so it's built at most once per frame, and not
        // at all on FACE-mode frames that skip the hand pass.
        val uprightBitmap by lazy { imageProxy.toBitmapInternal(rotation) }

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
                        gesture = classifyHandGesture(hand.map { HandPoint(it.x() * frameWidth, it.y() * frameHeight) })
                        if (gesture == HandGesture.OPEN_PALM) {
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
                        BallBox(b.left.toInt(), b.top.toInt(), b.right.toInt(), b.bottom.toInt())
                    }
                }
                Tasks.forResult(ballTracker.update(boxes).map { (box, id) ->
                    MainActivity.DetectedObjectInfo(Rect(box.left, box.top, box.right, box.bottom), id, label = "Ball")
                })
            } catch (e: Exception) {
                Log.e("CamX", "Ball detection error: ${e.message}")
                Tasks.forException(e)
            }
            // People, from the pose model first: it detects persons only and
            // gives body landmarks, so the aim point sits on the torso and the
            // skeleton can be drawn. On frames where it finds no one (typically a
            // subject too small in frame for it), the COCO "person" class takes
            // over, with ML Kit faces matched to each body to aim at the upper
            // body -- see bodyAimPoint(). Either way these are per-frame
            // detections with no IDs, so a BallTracker (generic box tracker)
            // supplies them for lock-on and coasting; both run synchronously
            // here on the executor (MediaPipe must stay on this thread).
            DetectionMode.BODY -> try {
                val mpImage = BitmapImageBuilder(uprightBitmap).build()
                val poses = getPoseLandmarker()?.detect(mpImage)?.landmarks().orEmpty().mapNotNull { pose ->
                    poseToBody(
                        pose.map { PosePoint(it.x() * frameWidth, it.y() * frameHeight, it.visibility().orElse(0f)) },
                        frameWidth, frameHeight
                    )
                }
                if (poses.isNotEmpty()) {
                    // update() returns the boxes in the order given.
                    val ids = personTracker.update(poses.map { it.box }).map { it.second }
                    Tasks.forResult(poses.zip(ids) { pose, id ->
                        val box = pose.box
                        MainActivity.DetectedObjectInfo(
                            Rect(box.left, box.top, box.right, box.bottom), id, label = "Person",
                            aimX = pose.aimX, aimY = pose.aimY,
                            skeleton = pose.skeleton.map { p -> p?.let { Offset(it.x, it.y) } }
                        )
                    })
                } else {
                    val detector = getPersonDetector() ?: throw IllegalStateException("Person detector unavailable")
                    val boxes = detector.detect(mpImage).detections().map { d ->
                        val b = d.boundingBox()
                        BallBox(b.left.toInt(), b.top.toInt(), b.right.toInt(), b.bottom.toInt())
                    }
                    val bodies = personTracker.update(boxes)
                    faceDetector.process(image).continueWith { t ->
                        val faces = if (t.isSuccessful) t.result.map { it.boundingBox } else emptyList()
                        bodies.map { (box, id) ->
                            val body = Rect(box.left, box.top, box.right, box.bottom)
                            // Largest face whose centre lies inside this body box.
                            val face = faces
                                .filter { body.contains(it.centerX(), it.centerY()) }
                                .maxByOrNull { it.width().toLong() * it.height().toLong() }
                            val (aimX, aimY) = bodyAimPoint(body, face)
                            MainActivity.DetectedObjectInfo(body, id, label = "Person", aimX = aimX, aimY = aimY, faceBox = face)
                        }
                    }
                }
            } catch (e: Exception) {
                Log.e("CamX", "Person detection error: ${e.message}")
                Tasks.forException(e)
            }
        }

        detectionTask
            .addOnSuccessListener { detectedObjects ->
                if (openPalmDetected && palmLockEnabled && lockedId == null) {
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
                            isLocked = it.trackingId == lockedId,
                            faceBox = it.faceBox,
                            skeleton = it.skeleton
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

                // Paused by the fist gesture: the subject is still found and drawn,
                // but falls through to the hold-still branch below.
                // With room-coordinate tracking on, the filters hold the subject's
                // position with the camera's own rotation taken out (see RoomFrame),
                // and the shift puts it back to get image positions. Off, the shift
                // is zero and the filters work in the image as before.
                val shift = roomFrame.shiftFor(imageProxy.imageInfo.timestamp, frameWidth, frameHeight, isFrontCamera)

                if (targetObject != null && !paused) {
                    rawX = targetObject.aimX ?: targetObject.boundingBox.exactCenterX()
                    rawY = targetObject.aimY ?: targetObject.boundingBox.exactCenterY()
                    filteredX = kalmanFilterX.update(rawX - shift.frameX, imageProxy.imageInfo.timestamp) + shift.frameX
                    filteredY = kalmanFilterY.update(rawY - shift.frameY, imageProxy.imageInfo.timestamp) + shift.frameY
                    predictedX = kalmanFilterX.predictFuture(predictionHorizonSeconds) + shift.sendX
                    predictedY = kalmanFilterY.predictFuture(predictionHorizonSeconds) + shift.sendY
                } else if (coasting && !paused) {
                    rawX = Float.NaN
                    rawY = Float.NaN
                    filteredX = if (kalmanFilterX.hasEstimate) kalmanFilterX.position + shift.frameX else frameWidth / 2f
                    filteredY = if (kalmanFilterY.hasEstimate) kalmanFilterY.position + shift.frameY else frameHeight / 2f
                    predictedX = kalmanFilterX.predictFuture(predictionHorizonSeconds) + shift.sendX
                    predictedY = kalmanFilterY.predictFuture(predictionHorizonSeconds) + shift.sendY
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

                // Tilt sign, verified on the rig (phone in portrait): the back camera
                // sends the frame's Y error as-is (positive = subject below centre,
                // the firmware's convention) and the front camera, which faces the
                // other way so the same tilt moves its view the opposite way, needs
                // it negated. Commit 2d9c997 swapped these and tilt then drove away
                // from the subject on both cameras.
                // The same errors without the prediction, for the hold-still band.
                val measuredErrX = ((filteredX - frameWidth / 2f) / (frameWidth / 2f)).coerceIn(-1f, 1f)
                var measuredErrY = ((filteredY - frameHeight / 2f) / (frameHeight / 2f)).coerceIn(-1f, 1f)

                if (isFrontCamera) {
                    errY = -errY
                    measuredErrY = -measuredErrY
                }

                val update = MainActivity.TrackingUpdate(
                    errX = errX,
                    errY = errY,
                    measuredErrX = measuredErrX,
                    measuredErrY = measuredErrY,
                    subjectSpeedX = if (roomFrame.active) abs(kalmanFilterX.velocity) / (frameWidth / 2f) else 0f,
                    subjectSpeedY = if (roomFrame.active && roomFrame.mode == RoomFrame.PAN_TILT) {
                        abs(kalmanFilterY.velocity) / (frameHeight / 2f)
                    } else {
                        0f
                    },
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
                    handChecked = shouldDetectHands,
                    gesture = gesture
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

/**
 * BODY-mode aim point for a person box: horizontally the body centre, vertically
 * pulled up towards the face so the head stays in frame with as much of the body
 * as fits (see FACE_AIM_OFFSET / BODY_AIM_FALLBACK_FRACTION).
 */
private fun bodyAimPoint(body: Rect, face: Rect?): Pair<Float, Float> {
    val aimY = if (face != null) {
        minOf(face.exactCenterY() + FACE_AIM_OFFSET * face.height(), body.exactCenterY())
    } else {
        body.top + BODY_AIM_FALLBACK_FRACTION * body.height()
    }
    return body.exactCenterX() to aimY
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
