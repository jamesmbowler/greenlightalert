package com.greenlightalert.trafficcam

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.provider.Settings
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.greenlightalert.trafficcam.ui.TrafficCamApp

@Composable
fun PermissionAwareTrafficCamApp() {
    val context = LocalContext.current
    val requiredPermissions = remember {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
            )
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
    }
    fun isGranted(permission: String): Boolean =
        ContextCompat.checkSelfPermission(context, permission) == PackageManager.PERMISSION_GRANTED

    var permissionStates by remember {
        mutableStateOf(requiredPermissions.associateWith(::isGranted))
    }
    var requestAttempted by remember { mutableStateOf(false) }
    val permissionsGranted = permissionStates.values.all { it }

    val launcher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions(),
    ) { result ->
        requestAttempted = true
        permissionStates = requiredPermissions.associateWith { permission ->
            result[permission] == true || isGranted(permission)
        }
    }

    LaunchedEffect(Unit) {
        if (!permissionsGranted) {
            requestAttempted = true
            launcher.launch(requiredPermissions)
        }
    }

    if (permissionsGranted) {
        TrafficCamApp(runtime = rememberTrafficCamRuntime())
    } else {
        MaterialTheme {
            Surface(modifier = Modifier.fillMaxSize()) {
                Card(modifier = Modifier.fillMaxWidth().padding(24.dp)) {
                    Column(
                        modifier = Modifier.fillMaxWidth().padding(20.dp),
                        verticalArrangement = Arrangement.spacedBy(12.dp),
                    ) {
                        Text("Bluetooth access required", style = MaterialTheme.typography.headlineSmall)
                        Text(
                            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                                "Grant Bluetooth scan and connect permissions so the app can discover and pair with the ESP32 camera."
                            } else {
                                "Grant location permission so Android can scan for BLE devices."
                            }
                        )
                        requiredPermissions.forEach { permission ->
                            val granted = permissionStates[permission] == true
                            val permissionLabel = permission.substringAfterLast('.')
                            val deniedPermanently =
                                requestAttempted &&
                                    !granted &&
                                    context is MainActivity &&
                                    !ActivityCompat.shouldShowRequestPermissionRationale(context, permission)
                            Text(
                                "$permissionLabel: " + when {
                                    granted -> "granted"
                                    deniedPermanently -> "denied - open settings"
                                    else -> "missing"
                                },
                                color = if (granted) {
                                    MaterialTheme.colorScheme.primary
                                } else {
                                    MaterialTheme.colorScheme.error
                                }
                            )
                        }
                        Button(onClick = { launcher.launch(requiredPermissions) }) {
                            Text("Grant permissions")
                        }
                        OutlinedButton(
                            onClick = {
                                val intent = Intent(
                                    Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                                    Uri.fromParts("package", context.packageName, null)
                                )
                                context.startActivity(intent)
                            }
                        ) {
                            Text("Open app settings")
                        }
                    }
                }
            }
        }
    }
}
