package com.example.tripodtracker

import android.Manifest
import android.content.ContentValues
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Rect
import android.os.Build
import android.os.Bundle
import android.provider.MediaStore
import android.net.nsd.NsdServiceInfo
import android.util.Log
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
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarkerResult
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
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
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.example.tripodtracker.ui.theme.TripodTrackerTheme
import com.google.mlkit.vision.common.InputImage
import com.google.mlkit.vision.objects.ObjectDetection
import com.google.mlkit.vision.objects.defaults.ObjectDetectorOptions
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.text.SimpleDateFormat
import java.util.*
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.hypot

class MainActivity : ComponentActivity() {

    private lateinit var cameraExecutor: ExecutorService
    private val kalmanFilter = KalmanFilter()
    private val udpSocket = DatagramSocket()
    lateinit var logManager: LogManager
    lateinit var nsdHelper: NsdHelper
    var handLandmarker: HandLandmarker? = null

    private var esp32Ip by mutableStateOf("10.179.76.141")
    private var udpPort by mutableIntStateOf(4210)
    private var discoveredServices = mutableStateListOf<NsdServiceInfo>()
    private var isLogging by mutableStateOf(false)
    private var currentScreen by mutableStateOf("camera")
    private var lockedTrackingId by mutableStateOf<Int?>(null)

    data class DetectedObjectInfo(
        val boundingBox: Rect,
        val trackingId: Int?,
        val label: String = "Person",
        val isLocked: Boolean = false
    )

    data class DetectionResult(
        val objects: List<DetectedObjectInfo>,
        val imageWidth: Int,
        val imageHeight: Int,
        val isFrontCamera: Boolean
    )

    private val permissions = arrayOf(
        Manifest.permission.CAMERA,
        Manifest.permission.RECORD_AUDIO
    )

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { results ->
        if (results[Manifest.permission.CAMERA] == true) {
            startCamera()
        } else {
            Toast.makeText(this, "Camera permission is required.", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        cameraExecutor = Executors.newSingleThreadExecutor()
        logManager = LogManager(this)
        nsdHelper = NsdHelper(this) { serviceInfo ->
            if (discoveredServices.none { it.serviceName == serviceInfo.serviceName }) {
                discoveredServices.add(serviceInfo)
            }
        }

        setupHandLandmarker()

        if (allPermissionsGranted()) {
            startCamera()
        } else {
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
        } catch (e: Exception) {
            Log.e("CamX", "HandLandmarker init failed", e)
        }
    }

    private fun allPermissionsGranted() = permissions.all {
        ContextCompat.checkSelfPermission(baseContext, it) == PackageManager.PERMISSION_GRANTED
    }

    private fun startCamera() {
        setContent {
            TripodTrackerTheme {
                CamXApp()
            }
        }
    }

    @Composable
    fun CamXApp() {
        if (currentScreen == "settings") {
            BackHandler { currentScreen = "camera" }
            ConnectionScreen(
                discoveredDevices = discoveredServices,
                currentIp = esp32Ip,
                currentPort = udpPort,
                onConnect = { ip, port ->
                    esp32Ip = ip
                    udpPort = port
                    currentScreen = "camera"
                },
                onTest = { ip, port ->
                    sendTestPacket(ip, port)
                },
                onDiscoveryStart = {
                    discoveredServices.clear()
                    nsdHelper.startDiscovery()
                },
                onDiscoveryStop = {
                    nsdHelper.stopDiscovery()
                }
            )
        } else {
            CameraPreviewScreen(
                cameraExecutor,
                kalmanFilter,
                isLogging = isLogging,
                onToggleLogging = {
                    isLogging = it
                    if (!it) logManager.saveLog()
                },
                onOpenSettings = { currentScreen = "settings" },
                lockedId = lockedTrackingId,
                onSetLockedId = { lockedTrackingId = it }
            ) { x, y ->
                sendUdpCommand(x, y)
                if (isLogging) {
                    logManager.log(x.toFloat(), y.toFloat())
                }
            }
        }
    }

    private fun sendTestPacket(ip: String, port: Int) {
        thread {
            try {
                val address = InetAddress.getByName(ip)
                val buffer = "PING".toByteArray()
                val packet = DatagramPacket(buffer, buffer.size, address, port)
                udpSocket.send(packet)
                runOnUiThread { Toast.makeText(this, "Test packet sent to $ip", Toast.LENGTH_SHORT).show() }
            } catch (e: Exception) {
                runOnUiThread { Toast.makeText(this, "Test failed: ${e.message}", Toast.LENGTH_SHORT).show() }
            }
        }
    }

    private fun sendUdpCommand(centerX: Int, centerY: Int) {
        thread {
            try {
                // Sending exact center pixel coordinates as requested
                val message = "X:$centerX,Y:$centerY"
                val buffer = message.toByteArray()
                val address = InetAddress.getByName(esp32Ip)
                val packet = DatagramPacket(buffer, buffer.size, address, udpPort)
                udpSocket.send(packet)
            } catch (e: Exception) {
                Log.e("UDP", "Failed to send packet", e)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraExecutor.shutdown()
        handLandmarker?.close()
        udpSocket.close()
    }
}

@Composable
fun CameraPreviewScreen(
    executor: ExecutorService,
    kalmanFilter: KalmanFilter,
    isLogging: Boolean,
    onToggleLogging: (Boolean) -> Unit,
    onOpenSettings: () -> Unit,
    lockedId: Int?,
    onSetLockedId: (Int?) -> Unit,
    onTargetDetected: (Int, Int) -> Unit
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
                text = if (lockedId != null) "LOCKED" else if (detectionResult != null) "Tracking..." else "Searching...",
                color = if (lockedId != null) Color.Cyan else Color.Green,
                modifier = Modifier.padding(top = 64.dp).align(Alignment.TopCenter),
                style = MaterialTheme.typography.headlineSmall
            )
            
            Canvas(modifier = Modifier.fillMaxSize()) {
                detectionResult?.let { result ->
                    val isFront = result.isFrontCamera
                    val scaleX = size.width / result.imageWidth
                    val scaleY = size.height / result.imageHeight

                    result.objects.forEach { obj ->
                        val isLocked = obj.trackingId == lockedId
                        // Only show the box if it's the locked one OR if nothing is locked (show prominent)
                        if (lockedId == null || isLocked) {
                            val left = if (isFront) size.width - (obj.boundingBox.right * scaleX) else obj.boundingBox.left * scaleX
                            val right = if (isFront) size.width - (obj.boundingBox.left * scaleX) else obj.boundingBox.right * scaleX
                            val top = obj.boundingBox.top * scaleY
                            val bottom = obj.boundingBox.bottom * scaleY

                            drawRect(
                                color = if (isLocked) Color.Cyan else Color.Green,
                                topLeft = Offset(left, top),
                                size = Size(right - left, bottom - top),
                                style = Stroke(width = if (isLocked) 8.dp.toPx() else 6.dp.toPx())
                            )
                        }
                    }
                }
            }
        }

        // --- UI Overlays ---
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            IconButton(onClick = onOpenSettings) {
                Icon(Icons.Default.Settings, contentDescription = "Settings", tint = Color.White)
            }
            
            Row {
                if (lockedId != null) {
                    IconButton(onClick = { onSetLockedId(null) }) {
                        Icon(Icons.Default.LockOpen, contentDescription = "Unlock", tint = Color.Cyan)
                    }
                }

                IconButton(onClick = { 
                    isTrackingEnabled = !isTrackingEnabled 
                    if (!isTrackingEnabled) {
                        onSetLockedId(null)
                        kalmanFilter.reset()
                        onTargetDetected(320, 240) // Default center
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
                    onSetLockedId(null)
                }) {
                    Icon(Icons.Filled.FlipCameraAndroid, contentDescription = "Flip", tint = Color.White)
                }
            }
        }

        Row(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 48.dp)
                .fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceEvenly,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(64.dp)
                    .clip(CircleShape)
                    .background(if (activeRecording != null) Color.Red else Color.White.copy(alpha = 0.5f))
                    .border(2.dp, Color.White, CircleShape)
                    .clickable {
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
                modifier = Modifier
                    .size(80.dp)
                    .clip(CircleShape)
                    .background(Color.White)
                    .border(4.dp, Color.Gray, CircleShape)
                    .clickable { takePhoto(context, imageCapture, executor) },
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

            val imageAnalyzer = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
                .also {
                it.setAnalyzer(executor) { imageProxy ->
                    if (isTrackingEnabled) {
                        val mainActivity = (context as MainActivity)
                        processImageProxy(
                            objectDetector, 
                            mainActivity.handLandmarker,
                            imageProxy, 
                            cameraSelector == CameraSelector.DEFAULT_FRONT_CAMERA,
                            kalmanFilter,
                            lockedId,
                            onSetLockedId
                        ) { x, y, result ->
                            onTargetDetected(x, y)
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
                Log.e("CameraX", "Binding failed", exc)
            }
        }
    }
}

@OptIn(ExperimentalGetImage::class)
private fun processImageProxy(
    detector: com.google.mlkit.vision.objects.ObjectDetector,
    handLandmarker: HandLandmarker?,
    imageProxy: ImageProxy,
    isFrontCamera: Boolean,
    kalmanFilter: KalmanFilter,
    lockedId: Int?,
    onSetLockedId: (Int?) -> Unit,
    onResult: (Int, Int, MainActivity.DetectionResult?) -> Unit
) {
    val bitmap = imageProxy.toBitmap()
    val mediaImage = imageProxy.image
    if (mediaImage != null) {
        val rotation = imageProxy.imageInfo.rotationDegrees
        val image = InputImage.fromMediaImage(mediaImage, rotation)

        // 1. Detect Hands for Gesture Lock
        var openPalmDetected = false
        var palmX = 0f
        var palmY = 0f
        
        handLandmarker?.let { landmarker ->
            val mpImage = BitmapImageBuilder(bitmap).build()
            val result = landmarker.detect(mpImage)
            if (result.landmarks().isNotEmpty()) {
                val hand = result.landmarks()[0]
                // Check for Open Palm: Fingers extended
                // Simplified check: tips higher than PIP joints (for vertical palm)
                // Landmarks: 8 (index tip), 12 (middle), 16 (ring), 20 (pinky)
                val isExtended = hand[8].y() < hand[6].y() && hand[12].y() < hand[10].y() && 
                                hand[16].y() < hand[14].y() && hand[20].y() < hand[18].y()
                if (isExtended) {
                    openPalmDetected = true
                    // Palm position (landmark 0 is wrist, 9 is middle base)
                    palmX = hand[9].x() * imageProxy.width
                    palmY = hand[9].y() * imageProxy.height
                }
            }
        }

        // 2. Detect Objects (People)
        detector.process(image)
            .addOnSuccessListener { detectedObjects ->
                if (detectedObjects.isNotEmpty()) {
                    val objectInfos = detectedObjects.map { 
                        MainActivity.DetectedObjectInfo(it.boundingBox, it.trackingId, isLocked = it.trackingId == lockedId)
                    }

                    // Handle Palm Locking
                    if (openPalmDetected && lockedId == null) {
                        val closest = detectedObjects.minByOrNull { obj ->
                            hypot(obj.boundingBox.centerX() - palmX, obj.boundingBox.centerY() - palmY)
                        }
                        if (closest != null && closest.trackingId != null) {
                            onSetLockedId(closest.trackingId)
                        }
                    }

                    // Select target: locked one or first one
                    val targetObject = if (lockedId != null) {
                        detectedObjects.find { it.trackingId == lockedId } ?: detectedObjects.first()
                    } else {
                        detectedObjects.first()
                    }

                    val rawCenterX = targetObject.boundingBox.exactCenterX()
                    val rawCenterY = targetObject.boundingBox.exactCenterY()

                    // Apply Kalman Filter for prediction
                    kalmanFilter.update(rawCenterX, System.currentTimeMillis())
                    val predictedX = kalmanFilter.predictFuture(0.1f)
                    
                    val result = MainActivity.DetectionResult(
                        objects = objectInfos,
                        imageWidth = imageProxy.width,
                        imageHeight = imageProxy.height,
                        isFrontCamera = isFrontCamera
                    )
                    onResult(predictedX.toInt(), rawCenterY.toInt(), result)
                } else {
                    onResult(imageProxy.width / 2, imageProxy.height / 2, null)
                }
            }
            .addOnCompleteListener {
                imageProxy.close()
            }
    } else {
        imageProxy.close()
    }
}

// Extension to convert ImageProxy to Bitmap efficiently for MediaPipe
@OptIn(ExperimentalGetImage::class)
private fun ImageProxy.toBitmap(): Bitmap {
    val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
    bitmap.copyPixelsFromBuffer(planes[0].buffer)
    return bitmap
}

@Composable
fun ConnectionScreen(
    discoveredDevices: List<NsdServiceInfo>,
    currentIp: String,
    currentPort: Int,
    onConnect: (String, Int) -> Unit,
    onTest: (String, Int) -> Unit,
    onDiscoveryStart: () -> Unit,
    onDiscoveryStop: () -> Unit
) {
    var ip by remember { mutableStateOf(currentIp) }
    var port by remember { mutableStateOf(currentPort.toString()) }

    LaunchedEffect(Unit) { onDiscoveryStart() }
    DisposableEffect(Unit) { onDispose { onDiscoveryStop() } }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("Tripod Connection", style = MaterialTheme.typography.headlineMedium, color = MaterialTheme.colorScheme.onBackground)
        Spacer(modifier = Modifier.height(24.dp))

        OutlinedTextField(
            value = ip,
            onValueChange = { ip = it },
            label = { Text("ESP32 IP Address") },
            modifier = Modifier.fillMaxWidth(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            colors = OutlinedTextFieldDefaults.colors(
                focusedTextColor = MaterialTheme.colorScheme.onBackground,
                unfocusedTextColor = MaterialTheme.colorScheme.onBackground
            )
        )

        Spacer(modifier = Modifier.height(8.dp))

        OutlinedTextField(
            value = port,
            onValueChange = { port = it },
            label = { Text("UDP Port") },
            modifier = Modifier.fillMaxWidth(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            colors = OutlinedTextFieldDefaults.colors(
                focusedTextColor = MaterialTheme.colorScheme.onBackground,
                unfocusedTextColor = MaterialTheme.colorScheme.onBackground
            )
        )

        Spacer(modifier = Modifier.height(16.dp))

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            Button(onClick = { onTest(ip, port.toIntOrNull() ?: 4210) }) { Text("Test Connection") }
            Button(onClick = { onConnect(ip, port.toIntOrNull() ?: 4210) }) { Text("Save & Connect") }
        }

        Spacer(modifier = Modifier.height(32.dp))
        Text("Discovered Devices", style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.onBackground)
        
        LazyColumn(modifier = Modifier.fillMaxWidth()) {
            items(discoveredDevices) { device ->
                Card(
                    modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp).clickable {
                        ip = device.host.hostAddress ?: ""
                        port = device.port.toString()
                    },
                    colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
                ) {
                    Column(modifier = Modifier.padding(16.dp)) {
                        Text(device.serviceName, style = MaterialTheme.typography.bodyLarge, color = MaterialTheme.colorScheme.onSurfaceVariant)
                        Text("${device.host.hostAddress}:${device.port}", style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.7f))
                    }
                }
            }
        }
    }
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
