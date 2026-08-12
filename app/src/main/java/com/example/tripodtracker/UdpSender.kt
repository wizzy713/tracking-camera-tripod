package com.example.tripodtracker

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.Executors

/**
 * Managed UDP sender that uses a single background thread to avoid network overhead.
 */
class UdpSender {
    private val executor = Executors.newSingleThreadExecutor()
    private val socket = DatagramSocket()
    
    private var targetIp: String = "10.179.76.141"
    private var targetPort: Int = 4210

    fun updateTarget(ip: String, port: Int) {
        targetIp = ip
        targetPort = port
    }

    fun send(message: String) {
        executor.execute {
            try {
                val address = InetAddress.getByName(targetIp)
                val buffer = message.toByteArray()
                val packet = DatagramPacket(buffer, buffer.size, address, targetPort)
                socket.send(packet)
            } catch (e: Exception) {
                Log.e("UdpSender", "Failed to send: ${e.message}")
            }
        }
    }

    fun close() {
        executor.shutdown()
        socket.close()
    }
}
