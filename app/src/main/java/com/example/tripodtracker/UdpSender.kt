package com.example.tripodtracker

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import android.os.SystemClock
import java.net.InetAddress
import java.net.NetworkInterface
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
 * The tripod's live PID gains, as reported back over UDP in reply to a
 * `CFG:...` (set) or `CFG?` (query) packet -- see the Experiment tab and
 * "CFG protocol" in firmware/README.md.
 */
data class TripodConfig(
    val kp: Float,
    val ki: Float,
    val kd: Float,
    val maxSpeedOffsetUs: Float,
    val deadzone: Float
)

/**
 * Managed UDP link to the tripod. A single background thread serializes sends
 * (see [send]); a second daemon thread drains replies and surfaces battery
 * telemetry via [onBattery]. Both share one [DatagramSocket] so the firmware's
 * reply (sent back to our source address) lands here without a second port.
 */
class UdpSender {
    private val executor = Executors.newSingleThreadExecutor()
    private val socket = DatagramSocket().apply { broadcast = true }

    @Volatile private var running = true
    private val receiver: Thread

    /** Invoked (on the receiver thread) whenever a battery packet is parsed. */
    @Volatile var onBattery: ((BatteryStatus) -> Unit)? = null

    /** Invoked (on the receiver thread) whenever a CFG reply is parsed. */
    @Volatile var onConfig: ((TripodConfig) -> Unit)? = null

    /**
     * Invoked (on the receiver thread) with the tripod's IP when it answers a
     * [discover] broadcast with `TRIPOD:<ip>`.
     */
    @Volatile var onTripodFound: ((String) -> Unit)? = null

    @Volatile private var targetIp: String = "10.47.140.33"
    @Volatile private var targetPort: Int = 4210

    /**
     * [SystemClock.elapsedRealtime] of the last packet received from the current
     * target, or 0 if none yet. The tripod pushes battery telemetry every ~2 s
     * once it hears from us, so a stale value means we're not connected.
     */
    @Volatile var lastRxFromTargetMs: Long = 0L
        private set

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
                    val fromIp = packet.address?.hostAddress
                    Log.d("UdpSender", "rx from $fromIp:${packet.port} -> \"$message\"")
                    if (fromIp == targetIp) lastRxFromTargetMs = SystemClock.elapsedRealtime()
                    when {
                        message.startsWith("TRIPOD") -> {
                            // Trust the packet's source address over the payload: it's
                            // the address that actually reached us.
                            if (fromIp != null) onTripodFound?.invoke(fromIp)
                        }
                        message.startsWith("BATT:") -> {
                            val status = parseBattery(message)
                            if (status != null) onBattery?.invoke(status)
                            else Log.w("UdpSender", "rx BATT packet malformed: \"$message\"")
                        }
                        message.startsWith("CFG:") -> {
                            val config = parseConfig(message)
                            if (config != null) onConfig?.invoke(config)
                            else Log.w("UdpSender", "rx CFG packet malformed: \"$message\"")
                        }
                        else -> Log.w("UdpSender", "rx packet not recognized")
                    }
                } catch (e: Exception) {
                    if (running) Log.w("UdpSender", "receive failed: ${e.message}")
                }
            }
        }
    }

    fun updateTarget(ip: String, port: Int) {
        if (ip != targetIp) lastRxFromTargetMs = 0L
        targetIp = ip
        targetPort = port
    }

    /**
     * Broadcasts `DISCOVER` on every active IPv4 network (each interface's
     * directed broadcast address, plus 255.255.255.255). A tripod on the same
     * network answers with `TRIPOD:<ip>`, surfaced via [onTripodFound].
     */
    fun discover() {
        executor.execute {
            try {
                val targets = mutableSetOf<InetAddress>(InetAddress.getByName("255.255.255.255"))
                NetworkInterface.getNetworkInterfaces()?.toList().orEmpty()
                    .filter { it.isUp && !it.isLoopback }
                    .flatMap { it.interfaceAddresses }
                    .mapNotNullTo(targets) { it.broadcast }
                val buffer = "DISCOVER".toByteArray()
                for (address in targets) {
                    try {
                        socket.send(DatagramPacket(buffer, buffer.size, address, targetPort))
                    } catch (e: Exception) {
                        Log.w("UdpSender", "discover to ${address.hostAddress} failed: ${e.message}")
                    }
                }
            } catch (e: Exception) {
                Log.w("UdpSender", "discover failed: ${e.message}")
            }
        }
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

    /**
     * Parses `CFG:KP:<v>,KI:<v>,KD:<v>,MS:<v>,DZ:<v>` (same `KEY:value` convention
     * as [parseBattery]). Returns null unless all five gains are present.
     */
    private fun parseConfig(message: String): TripodConfig? {
        if (!message.startsWith("CFG:")) return null
        val fields = message.removePrefix("CFG:").split(",").mapNotNull { part ->
            val kv = part.split(":", limit = 2)
            if (kv.size == 2) kv[0].trim() to kv[1].trim() else null
        }.toMap()
        return TripodConfig(
            kp = fields["KP"]?.toFloatOrNull() ?: return null,
            ki = fields["KI"]?.toFloatOrNull() ?: return null,
            kd = fields["KD"]?.toFloatOrNull() ?: return null,
            maxSpeedOffsetUs = fields["MS"]?.toFloatOrNull() ?: return null,
            deadzone = fields["DZ"]?.toFloatOrNull() ?: return null
        )
    }
}
