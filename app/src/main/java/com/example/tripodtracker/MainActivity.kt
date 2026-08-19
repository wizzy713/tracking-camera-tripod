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
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
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
import com.google.mlkit.vision.common.InputImage
import com.google.mlkit.vision.objects.ObjectDetection
import com.google.mlkit.vision.objects.ObjectDetector
import com.google.mlkit.vision.objects.defaults.ObjectDetectorOptions
import kotlinx.coroutines.awaitCancellation
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
private const val PREDICTION_HORIZON_SECONDS = 0.1f

// How many consecutive frames a locked target may go unmatched before the lock
// is released. During this window the system coasts on the Kalman prediction
// instead of jumping to an arbitrary detection.
private const val MAX_COAST_FRAMES = 15

class MainActivity : ComponentActivity() {

    private lateinit var cameraExecutor: ExecutorService
    private val kalmanFilterX = KalmanFilter()
    private val kalmanFilterY = KalmanFilter()
    private val udpSender = UdpSender()
    lateinit var logManager: LogManager
    @Volatile var handLandmarker: HandLandmarker? = null

    private var esp32Ip by mutableStateOf("10.179.76.141")
    private var udpPort by mutableIntStateOf(4210)
    private var isLogging by mutableStateOf(false)
    private var currentScreen by mutableStateOf("camera")
    private var lockedId by mutableStateOf<Int?>(null)
    private var permissionsGranted by mutableStateOf(false)
    private var packetSeq = 0L

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

        if (currentScreen == "settings") {
            BackHandler { currentScreen = "camera" }
            ConnectionScreen(
                currentIp = esp32Ip,
                currentPort = udpPort,
                isLogging = isLogging,
                onToggleLogging = {
                    isLogging = it
                    if (!it) logManager.saveLog()
                },
                onConnect = { ip, port ->
                    esp32Ip = ip
                    udpPort = port
                    currentScreen = "camera"
                },
                onTest = { ip, port, msg ->
                    udpSender.updateTarget(ip, port)
                    udpSender.send(msg)
                    Toast.makeText(this, "Test packet sent to $ip", Toast.LENGTH_SHORT).show()
                }
            )
        } else {
            CameraPreviewScreen(
                cameraExecutor,
                kalmanFilterX,
                kalmanFilterY,
                handLandmarker,
                isLogging = isLogging,
                onToggleLogging = {
                    isLogging = it
                    if (!it) logManager.saveLog()
                },
                onOpenSettings = { currentScreen = "settings" },
                lockedId = lockedId,
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
                        dtSeconds = update.dtSeconds
                    )
                }
            }
        }
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
    lockedId: Int?,
    onUnlock: () -> Unit,
    onTargetUpdate: (Int?) -> Unit,
    onTargetDetected: (MainActivity.TrackingUpdate) -> Unit
) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val previewView = remember { PreviewView(context) }

    var cameraSelector by remember { mutableStateOf(CameraSelector.DEFAULT_BACK_CAMERA) }
    var detectionResult by remember { mutableStateOf<MainActivity.DetectionResult?>(null) }
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
            }
        }

        Row(
            modifier = Modifier.fillMaxWidth().padding(16.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            IconButton(onClick = onOpenSettings) {
                Icon(Icons.Default.Settings, contentDescription = "Settings", tint = Color.White)
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
                }) {
                    Icon(Icons.Filled.FlipCameraAndroid, contentDescription = "Flip", tint = Color.White)
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

            val options = ObjectDetectorOptions.Builder()
                .setDetectorMode(ObjectDetectorOptions.STREAM_MODE)
                .enableMultipleObjects()
                .build()
            val objectDetector = ObjectDetection.getClient(options)

            var frameCounter = 0
            var lostFrameCount = 0
            val imageAnalyzer = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
                .also {
                it.setAnalyzer(executor) { imageProxy ->
                    if (isTrackingEnabled) {
                        frameCounter++
                        processImageProxy(
                            objectDetector,
                            landmarker,
                            imageProxy,
                            cameraSelector == CameraSelector.DEFAULT_FRONT_CAMERA,
                            kalmanFilterX,
                            kalmanFilterY,
                            lockedId,
                            lostFrameCount,
                            frameCounter % 5 == 0,
                            onTargetUpdate,
                            onLostFrameCountChanged = { lostFrameCount = it }
                        ) { update, result ->
                            onTargetDetected(update)
                            detectionResult = result
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
                objectDetector.close()
            }
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
            Button(onClick = { onTest(ip, port.toIntOrNull() ?: 4210, testMessage) }) { Text("Test Connection") }
            Button(onClick = { onConnect(ip, port.toIntOrNull() ?: 4210) }) { Text("Save & Connect") }
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
    detector: ObjectDetector,
    handLandmarker: HandLandmarker?,
    imageProxy: ImageProxy,
    isFrontCamera: Boolean,
    kalmanFilterX: KalmanFilter,
    kalmanFilterY: KalmanFilter,
    lockedId: Int?,
    lostFrameCount: Int,
    shouldDetectHands: Boolean,
    onSetLockedId: (Int?) -> Unit,
    onLostFrameCountChanged: (Int) -> Unit,
    onResult: (MainActivity.TrackingUpdate, MainActivity.DetectionResult?) -> Unit
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

        // Only detect hands periodically to save resources and prevent crashes
        if (shouldDetectHands) {
            handLandmarker?.let { landmarker ->
                try {
                    // Rotate to the same upright frame ML Kit uses, so palm
                    // coordinates and bounding-box coordinates are comparable.
                    val bitmap = imageProxy.toBitmapInternal(rotation)
                    val mpImage = BitmapImageBuilder(bitmap).build()
                    val result = landmarker.detect(mpImage)
                    if (result.landmarks().isNotEmpty()) {
                        val hand = result.landmarks()[0]
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

        detector.process(image)
            .addOnSuccessListener { detectedObjects ->
                val objectInfos = detectedObjects.map {
                    MainActivity.DetectedObjectInfo(
                        it.boundingBox,
                        it.trackingId,
                        isLocked = it.trackingId == lockedId
                    )
                }

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

                if (targetObject != null) {
                    rawX = targetObject.boundingBox.exactCenterX()
                    rawY = targetObject.boundingBox.exactCenterY()
                    filteredX = kalmanFilterX.update(rawX, imageProxy.imageInfo.timestamp)
                    filteredY = kalmanFilterY.update(rawY, imageProxy.imageInfo.timestamp)
                } else {
                    // No fresh measurement this frame -- coast on the last estimate
                    // (locked target temporarily occluded/misdetected) rather than
                    // snapping to an arbitrary object or the frame centre.
                    rawX = Float.NaN
                    rawY = Float.NaN
                    filteredX = if (kalmanFilterX.hasEstimate) kalmanFilterX.position else frameWidth / 2f
                    filteredY = if (kalmanFilterY.hasEstimate) kalmanFilterY.position else frameHeight / 2f
                }

                val predictedX = kalmanFilterX.predictFuture(PREDICTION_HORIZON_SECONDS)
                val predictedY = kalmanFilterY.predictFuture(PREDICTION_HORIZON_SECONDS)

                var errX = (predictedX - frameWidth / 2f) / (frameWidth / 2f)
                if (isFrontCamera) errX = -errX // Mirror to match the mirrored preview / user's real-world left-right.
                errX = errX.coerceIn(-1f, 1f)
                val errY = ((predictedY - frameHeight / 2f) / (frameHeight / 2f)).coerceIn(-1f, 1f)

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
                    isFrontCamera = isFrontCamera
                )
                onResult(update, if (detectedObjects.isEmpty()) null else result)
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
