package com.example.tripodtracker

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.util.Log

/**
 * Handles Network Service Discovery safely without overlapping resolves.
 */
class NsdHelper(context: Context, private val onServiceResolved: (NsdServiceInfo) -> Unit) {

    private val nsdManager = context.getSystemService(Context.NSD_SERVICE) as NsdManager
    private val serviceType = "_arduino._tcp."
    private var isDiscoveryActive = false
    private var isResolving = false

    private val discoveryListener = object : NsdManager.DiscoveryListener {
        override fun onDiscoveryStarted(regType: String) {
            Log.d("NSD", "Service discovery started")
            isDiscoveryActive = true
        }

        override fun onServiceFound(service: NsdServiceInfo) {
            Log.d("NSD", "Service found: ${service.serviceName}")
            if (service.serviceName.contains("Tripod") || service.serviceType.contains("arduino")) {
                if (!isResolving) {
                    isResolving = true
                    try {
                        nsdManager.resolveService(service, object : NsdManager.ResolveListener {
                            override fun onResolveFailed(serviceInfo: NsdServiceInfo, errorCode: Int) {
                                Log.e("NSD", "Resolve failed: $errorCode")
                                isResolving = false
                            }

                            override fun onServiceResolved(serviceInfo: NsdServiceInfo) {
                                Log.d("NSD", "Service resolved: ${serviceInfo.host}:${serviceInfo.port}")
                                onServiceResolved(serviceInfo)
                                isResolving = false
                            }
                        })
                    } catch (e: Exception) {
                        Log.e("NSD", "Resolve error: ${e.message}")
                        isResolving = false
                    }
                }
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
        }

        override fun onStopDiscoveryFailed(serviceType: String, errorCode: Int) {
            Log.e("NSD", "Stop discovery failed: $errorCode")
            isDiscoveryActive = false
        }
    }

    fun startDiscovery() {
        if (!isDiscoveryActive) {
            try {
                isResolving = false
                nsdManager.discoverServices(serviceType, NsdManager.PROTOCOL_DNS_SD, discoveryListener)
            } catch (e: Exception) {
                Log.e("NSD", "Start discovery error: ${e.message}")
            }
        }
    }

    fun stopDiscovery() {
        if (isDiscoveryActive) {
            try {
                nsdManager.stopServiceDiscovery(discoveryListener)
            } catch (e: Exception) {
                Log.e("NSD", "Error stopping discovery: ${e.message}")
            }
        }
    }
}
