package com.example.tripodtracker

import android.Manifest
import android.content.ContentValues
import android.content.pm.PackageManager
import android.graphics.Rect
import android.os.Build
import android.os.Bundle
import android.provider.MediaStore
import android.util.Log
import android.widget.Toast
import androidx.activity.ComponentActivity
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
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
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

class MainActivity : ComponentActivity() {

    private lateinit var cameraExecutor: ExecutorService
    private val udpSocket = DatagramSocket()
    private val esp32Ip = "10.179.76.141" // ESP32 IP address
    private val udpPort = 4210

    data class DetectedObjectInfo(
        val boundingBox: Rect,
        val trackingId: Int?,
        val label: String = "Object"
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

        if (allPermissionsGranted()) {
            startCamera()
        } else {
            requestPermissionLauncher.launch(permissions)
        }
    }

    private fun allPermissionsGranted() = permissions.all {
        ContextCompat.checkSelfPermission(baseContext, it) == PackageManager.PERMISSION_GRANTED
    }

    private fun startCamera() {
        setContent {
            TripodTrackerTheme {
                CameraPreviewScreen(cameraExecutor) { angle ->
                    sendUdpCommand(angle)
                }
            }
        }
    }

    private fun sendUdpCommand(panAngle: Int) {
        thread {
            try {
                val message = panAngle.toString()
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
        udpSocket.close()
    }
}

@Composable
fun CameraPreviewScreen(
    executor: ExecutorService,
    onTargetDetected: (Int) -> Unit
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
            Canvas(modifier = Modifier.fillMaxSize()) {
                detectionResult?.let { result ->
                    val isFront = result.isFrontCamera
                    val scaleX = size.width / result.imageWidth
                    val scaleY = size.height / result.imageHeight

                    // Only show the first (most prominent) object in a green box
                    result.objects.firstOrNull()?.let { obj ->
                        val left = if (isFront) size.width - (obj.boundingBox.right * scaleX) else obj.boundingBox.left * scaleX
                        val right = if (isFront) size.width - (obj.boundingBox.left * scaleX) else obj.boundingBox.right * scaleX
                        val top = obj.boundingBox.top * scaleY
                        val bottom = obj.boundingBox.bottom * scaleY

                        drawRect(
                            color = Color.Green,
                            topLeft = Offset(left, top),
                            size = Size(right - left, bottom - top),
                            style = Stroke(width = 6.dp.toPx())
                        )
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
            IconButton(onClick = {
                flashMode = when (flashMode) {
                    ImageCapture.FLASH_MODE_OFF -> ImageCapture.FLASH_MODE_ON
                    ImageCapture.FLASH_MODE_ON -> ImageCapture.FLASH_MODE_AUTO
                    else -> ImageCapture.FLASH_MODE_OFF
                }
                imageCapture.flashMode = flashMode
            }) {
                Icon(
                    imageVector = when (flashMode) {
                        ImageCapture.FLASH_MODE_ON -> Icons.Filled.FlashOn
                        ImageCapture.FLASH_MODE_AUTO -> Icons.Filled.FlashAuto
                        else -> Icons.Filled.FlashOff
                    },
                    contentDescription = "Flash",
                    tint = Color.White
                )
            }
            
            Row {
                IconButton(onClick = { 
                    isTrackingEnabled = !isTrackingEnabled 
                    if (!isTrackingEnabled) {
                        detectionResult = null
                        onTargetDetected(90) // Center motor when off
                    }
                }) {
                    Icon(
                        imageVector = if (isTrackingEnabled) Icons.Filled.TrackChanges else Icons.Filled.LocationDisabled,
                        contentDescription = "Tracking",
                        tint = if (isTrackingEnabled) Color.Green else Color.White
                    )
                }
                
                IconButton(onClick = {
                    cameraSelector = if (cameraSelector == CameraSelector.DEFAULT_BACK_CAMERA) CameraSelector.DEFAULT_FRONT_CAMERA else CameraSelector.DEFAULT_BACK_CAMERA
                    detectionResult = null
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
                .build()
                .also {
                it.setAnalyzer(executor) { imageProxy ->
                    if (isTrackingEnabled) {
                        processImageProxy(objectDetector, imageProxy, cameraSelector == CameraSelector.DEFAULT_FRONT_CAMERA) { angle, result ->
                            onTargetDetected(angle)
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
    imageProxy: ImageProxy,
    isFrontCamera: Boolean,
    onResult: (Int, MainActivity.DetectionResult?) -> Unit
) {
    val mediaImage = imageProxy.image
    if (mediaImage != null) {
        val rotation = imageProxy.imageInfo.rotationDegrees
        val image = InputImage.fromMediaImage(mediaImage, rotation)

        detector.process(image)
            .addOnSuccessListener { detectedObjects ->
                if (detectedObjects.isNotEmpty()) {
                    val objectInfos = detectedObjects.map { 
                        MainActivity.DetectedObjectInfo(it.boundingBox, it.trackingId)
                    }

                    // For now, track the first object
                    val targetObject = detectedObjects.first()
                    val targetCenterX = targetObject.boundingBox.exactCenterX()
                    
                    val isRotated = rotation == 90 || rotation == 270
                    val imageWidth = if (isRotated) imageProxy.height else imageProxy.width
                    val imageHeight = if (isRotated) imageProxy.width else imageProxy.height

                    var mappedAngle = ((targetCenterX / imageProxy.width) * 180).toInt()
                    mappedAngle = 180 - mappedAngle
                    
                    val result = MainActivity.DetectionResult(
                        objects = objectInfos,
                        imageWidth = imageWidth,
                        imageHeight = imageHeight,
                        isFrontCamera = isFrontCamera
                    )
                    onResult(mappedAngle, result)
                } else {
                    onResult(90, null)
                }
            }
            .addOnFailureListener {
                onResult(90, null)
            }
            .addOnCompleteListener {
                imageProxy.close()
            }
    } else {
        imageProxy.close()
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
            (context as? MainActivity)?.runOnUiThread {
                Toast.makeText(context, "Photo saved!", Toast.LENGTH_SHORT).show()
            }
        }
        override fun onError(exc: ImageCaptureException) {
            Log.e("CamX", "Photo capture failed", exc)
        }
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
                (context as? MainActivity)?.runOnUiThread {
                    Toast.makeText(context, "Video saved!", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }
}
