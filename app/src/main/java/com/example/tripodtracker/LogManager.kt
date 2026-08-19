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

    fun log(x: Float, y: Float) {
        val timestamp = dateFormat.format(Date())
        synchronized(lock) {
            data.add("$timestamp,$x,$y")
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
                    writer.write("Timestamp,CenterX,CenterY\n")
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
