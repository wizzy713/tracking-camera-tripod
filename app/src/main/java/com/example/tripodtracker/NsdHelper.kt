package com.example.tripodtracker

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.util.Log

class NsdHelper(context: Context, private val onServiceResolved: (NsdServiceInfo) -> Unit) {

    private val nsdManager = context.getSystemService(Context.NSD_SERVICE) as NsdManager
    private val serviceType = "_arduino._tcp."
    private var isDiscoveryActive = false

    private val discoveryListener = object : NsdManager.DiscoveryListener {
        override fun onDiscoveryStarted(regType: String) {
            Log.d("NSD", "Service discovery started")
            isDiscoveryActive = true
        }

        override fun onServiceFound(service: NsdServiceInfo) {
            Log.d("NSD", "Service found: ${service.serviceName}")
            if (service.serviceName.contains("Tripod") || service.serviceType.contains(serviceType.removeSuffix("."))) {
                nsdManager.resolveService(service, resolveListener)
            }
        }

        override fun onServiceLost(service: NsdServiceInfo) {
            Log.d("NSD", "Service lost: ${service.serviceName}")
        }

        override fun onDiscoveryStopped(regType: String) {
            Log.d("NSD", "Discovery stopped")
            isDiscoveryActive = false
        }

        override fun onStartDiscoveryFailed(serviceType: String, errorCode: Int) {
            Log.e("NSD", "Discovery failed: $errorCode")
            isDiscoveryActive = false
            try { nsdManager.stopServiceDiscovery(this) } catch (e: Exception) {}
        }

        override fun onStopDiscoveryFailed(serviceType: String, errorCode: Int) {
            Log.e("NSD", "Stop discovery failed: $errorCode")
            isDiscoveryActive = false
        }
    }

    private val resolveListener = object : NsdManager.ResolveListener {
        override fun onResolveFailed(serviceInfo: NsdServiceInfo, errorCode: Int) {
            Log.e("NSD", "Resolve failed: $errorCode")
        }

        override fun onServiceResolved(serviceInfo: NsdServiceInfo) {
            Log.d("NSD", "Service resolved: ${serviceInfo.host}:${serviceInfo.port}")
            onServiceResolved(serviceInfo)
        }
    }

    fun startDiscovery() {
        if (!isDiscoveryActive) {
            try {
                nsdManager.discoverServices(serviceType, NsdManager.PROTOCOL_DNS_SD, discoveryListener)
            } catch (e: Exception) {
                Log.e("NSD", "Start discovery error", e)
            }
        }
    }

    fun stopDiscovery() {
        if (isDiscoveryActive) {
            try {
                nsdManager.stopServiceDiscovery(discoveryListener)
            } catch (e: Exception) {
                Log.e("NSD", "Error stopping discovery", e)
            }
        }
    }
}
