package com.example.tripodtracker

import android.content.ContentValues
import android.content.Context
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import android.util.Log
import android.widget.Toast
import java.io.OutputStreamWriter
import java.text.SimpleDateFormat
import java.util.*

class LogManager(private val context: Context) {

    // log() runs on the frame-analysis executor thread while saveLog()/clear() run on
    // the UI thread, so access to `data` must be synchronized.
    private val lock = Any()
    private val data = mutableListOf<String>()
    private val dateFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.getDefault())

    /**
     * @param frameTimestampNanos Monotonic frame-capture timestamp (ImageProxy.imageInfo.timestamp), not wall-clock.
     * @param rawX/@param rawY Raw detection centre in pixels; NaN if this frame had no measurement (coasting).
     * @param filteredX/@param filteredY Kalman position estimate in pixels.
     * @param velocityX/@param velocityY Kalman velocity estimate in px/s -- lets you recompute
     *   predictions at any horizon offline (predicted = filtered + velocity * horizonSeconds)
     *   for a prediction-horizon sweep without re-running the app.
     * @param dtSeconds Time since the previous filter update, in seconds.
     */
    fun log(
        frameTimestampNanos: Long,
        seq: Long,
        detectionCount: Int,
        rawX: Float,
        rawY: Float,
        filteredX: Float,
        filteredY: Float,
        velocityX: Float,
        velocityY: Float,
        dtSeconds: Float
    ) {
        val timestamp = dateFormat.format(Date())
        val line = "$timestamp,$frameTimestampNanos,$seq,$detectionCount," +
            "$rawX,$rawY,$filteredX,$filteredY,$velocityX,$velocityY,$dtSeconds"
        synchronized(lock) {
            data.add(line)
        }
    }

    fun saveLog() {
        val snapshot = synchronized(lock) {
            if (data.isEmpty()) null else data.toList().also { data.clear() }
        }
        if (snapshot == null) {
            Toast.makeText(context, "No data to save", Toast.LENGTH_SHORT).show()
            return
        }

        val fileName = "TrackingLog_${SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(Date())}.csv"
        val contentValues = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, fileName)
            put(MediaStore.MediaColumns.MIME_TYPE, "text/csv")
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                put(MediaStore.MediaColumns.RELATIVE_PATH, Environment.DIRECTORY_DOCUMENTS + "/CamX_Logs")
            }
        }

        val resolver = context.contentResolver
        val uri = resolver.insert(MediaStore.Files.getContentUri("external"), contentValues)

        uri?.let {
            resolver.openOutputStream(it)?.use { outputStream ->
                OutputStreamWriter(outputStream).use { writer ->
                    writer.write(
                        "Timestamp,FrameTimestampNanos,Seq,DetectionCount," +
                            "RawX,RawY,FilteredX,FilteredY,VelocityX,VelocityY,DtSeconds\n"
                    )
                    snapshot.forEach { line ->
                        writer.write("$line\n")
                    }
                }
            }
            Toast.makeText(context, "Log saved to Documents/CamX_Logs", Toast.LENGTH_LONG).show()
        } ?: Log.e("LogManager", "Failed to create log file")
    }

    fun clear() {
        synchronized(lock) {
            data.clear()
        }
    }
}
