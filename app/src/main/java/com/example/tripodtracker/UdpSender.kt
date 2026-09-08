package com.example.tripodtracker

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.Executors
import kotlin.concurrent.thread

/**
 * Battery telemetry pushed back by the firmware (INA219 fuel gauge). Arrives as
 * a UDP reply on the same socket the error packets are sent from, roughly every
 * 2 s while the tripod has heard from the app.
 *
 * @param percent      state of charge, 0-100
 * @param millivolts   pack terminal voltage (2S Li-ion, ~6000-8400)
 * @param milliamps    pack current, positive = discharging
 * @param whRemaining  estimated watt-hours left (of a 9.62 Wh pack)
 */
data class BatteryStatus(
    val percent: Int,
    val millivolts: Int,
    val milliamps: Int,
    val whRemaining: Float
)

/**
 * Managed UDP link to the tripod. A single background thread serializes sends
 * (see [send]); a second daemon thread drains replies and surfaces battery
 * telemetry via [onBattery]. Both share one [DatagramSocket] so the firmware's
 * reply (sent back to our source address) lands here without a second port.
 */
class UdpSender {
    private val executor = Executors.newSingleThreadExecutor()
    private val socket = DatagramSocket()

    @Volatile private var running = true
    private val receiver: Thread

    /** Invoked (on the receiver thread) whenever a battery packet is parsed. */
    @Volatile var onBattery: ((BatteryStatus) -> Unit)? = null

    private var targetIp: String = "10.179.76.141"
    private var targetPort: Int = 4210

    private var sentCount = 0L

    init {
        Log.i("UdpSender", "listening for replies on local UDP port ${socket.localPort}")
        receiver = thread(name = "UdpReceiver", isDaemon = true) {
            val buffer = ByteArray(256)
            while (running) {
                try {
                    val packet = DatagramPacket(buffer, buffer.size)
                    socket.receive(packet)
                    val message = String(packet.data, 0, packet.length).trim()
                    Log.d("UdpSender", "rx from ${packet.address?.hostAddress}:${packet.port} -> \"$message\"")
                    val status = parseBattery(message)
                    if (status != null) {
                        onBattery?.invoke(status)
                    } else {
                        Log.w("UdpSender", "rx packet not recognized as battery telemetry")
                    }
                } catch (e: Exception) {
                    if (running) Log.w("UdpSender", "receive failed: ${e.message}")
                }
            }
        }
    }

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
                // Log the first send and then every ~150th (~5 s at 30 Hz) so the
                // log shows the link is alive without flooding.
                if (sentCount % 150 == 0L) {
                    Log.d("UdpSender", "tx #$sentCount to $targetIp:$targetPort -> \"$message\"")
                }
                sentCount++
            } catch (e: Exception) {
                Log.e("UdpSender", "Failed to send to $targetIp:$targetPort: ${e.message}")
            }
        }
    }

    fun close() {
        running = false
        executor.shutdown()
        socket.close() // unblocks the receiver's blocking receive()
    }

    /**
     * Parses `BATT:<pct>,MV:<millivolts>,MA:<milliamps>,WH:<wh>` (the firmware's
     * `KEY:value` comma-separated convention). Returns null for anything else.
     */
    private fun parseBattery(message: String): BatteryStatus? {
        if (!message.startsWith("BATT:")) return null
        val fields = message.split(",").mapNotNull { part ->
            val kv = part.split(":", limit = 2)
            if (kv.size == 2) kv[0].trim() to kv[1].trim() else null
        }.toMap()
        val percent = fields["BATT"]?.toIntOrNull() ?: return null
        return BatteryStatus(
            percent = percent.coerceIn(0, 100),
            millivolts = fields["MV"]?.toIntOrNull() ?: 0,
            milliamps = fields["MA"]?.toIntOrNull() ?: 0,
            whRemaining = fields["WH"]?.toFloatOrNull() ?: 0f
        )
    }
}
