package com.gasrobot.monitor.ui

import android.content.Intent
import android.net.Uri
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.animation.animateColor
import androidx.compose.animation.core.*
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.gasrobot.monitor.data.model.EmergencyInfo
import com.gasrobot.monitor.data.model.EmergencyKind
import com.gasrobot.monitor.ui.components.SectionCard
import com.gasrobot.monitor.ui.preview.PreviewData
import com.gasrobot.monitor.ui.theme.AppColors
import com.gasrobot.monitor.ui.viewmodel.MonitoringViewModel
import com.gasrobot.monitor.ui.viewmodel.MonitoringViewModelFactory

/**
 * This is what actually gets put in front of the user's face — via fullScreenIntent, it opens
 * on top of everything, including the lock screen. The siren + vibration are owned by
 * MonitoringViewModel (backed by the same AlarmController the foreground service uses), so this
 * Activity's only jobs are: (1) unlock-screen framework calls Compose can't do itself, and
 * (2) render EmergencyScreen from uiState and forward its two actions to the ViewModel/Android.
 * There's deliberately no back-button/swipe dismiss that silently clears the alarm.
 */
class EmergencyActivity : ComponentActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setShowWhenLockedCompat()

        setContent {
            val viewModel: MonitoringViewModel = viewModel(factory = MonitoringViewModelFactory(this))
            val uiState by viewModel.uiState.collectAsState()

            MaterialTheme {
                EmergencyScreen(
                    info = uiState.emergency,
                    onAcknowledge = {
                        viewModel.acknowledgeEmergency()
                        val open = Intent(this, MainActivity::class.java).apply {
                            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP
                            putExtra("route", "evac")
                        }
                        startActivity(open)
                        finish()
                    },
                    onCall119 = {
                        startActivity(Intent(Intent.ACTION_DIAL, Uri.parse("tel:119")))
                    }
                )
            }
        }
    }

    private fun setShowWhenLockedCompat() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
            setShowWhenLocked(true)
            setTurnScreenOn(true)
        }
    }
}

@Composable
private fun EmergencyScreen(info: EmergencyInfo, onAcknowledge: () -> Unit, onCall119: () -> Unit) {
    val transition = rememberInfiniteTransition(label = "alarmPulse")
    val bg by transition.animateColor(
        initialValue = AppColors.AlarmPulseA,
        targetValue = AppColors.AlarmPulseB,
        animationSpec = infiniteRepeatable(tween(700), RepeatMode.Reverse),
        label = "bg"
    )

    Box(
        Modifier
            .fillMaxSize()
            .background(bg)
            .padding(horizontal = 26.dp, vertical = 40.dp)
    ) {
        Column(Modifier.fillMaxSize(), horizontalAlignment = Alignment.CenterHorizontally) {
            val isFire = info.kind == EmergencyKind.FIRE
            Text(
                if (isFire) "FIRE DETECTED" else "GAS LEAK DETECTED",
                color = Color.White.copy(alpha = 0.9f), fontSize = 14.sp, fontWeight = FontWeight.Bold, letterSpacing = 4.sp
            )
            Spacer(Modifier.height(6.dp))
            Text(if (isFire) "화재 감지" else "가스 누출 감지", color = Color.White, fontSize = 34.sp, fontWeight = FontWeight.Black)
            Spacer(Modifier.height(10.dp))
            Text("${info.zoneLabel} · ${info.gasType}", color = Color.White.copy(alpha = 0.95f), fontSize = 17.sp)

            Spacer(Modifier.height(24.dp))
            InfoPanel(info)

            Spacer(Modifier.height(12.dp))
            Box(
                Modifier.fillMaxWidth().background(Color.Black.copy(alpha = 0.22f), MaterialTheme.shapes.large).padding(vertical = 20.dp),
                contentAlignment = Alignment.Center
            ) {
                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    Text("현재 농도", color = Color.White.copy(alpha = 0.85f), fontSize = 12.sp)
                    Text("${info.ppm}", color = Color.White, fontSize = 40.sp, fontWeight = FontWeight.Bold)
                }
            }

            Spacer(Modifier.height(16.dp))
            Text("진동 · 사이렌 작동 중 — 관리자 호출됨", color = Color.White.copy(alpha = 0.9f), fontSize = 12.sp)

            Spacer(Modifier.weight(1f))

            Button(
                onClick = onAcknowledge,
                modifier = Modifier.fillMaxWidth().height(56.dp),
                colors = ButtonDefaults.buttonColors(containerColor = Color.White, contentColor = AppColors.AlarmPulseA)
            ) {
                Text("상황 확인하기 · 지도 보기", fontWeight = FontWeight.Black, fontSize = 17.sp)
            }
            Spacer(Modifier.height(11.dp))
            OutlinedButton(
                onClick = onCall119,
                modifier = Modifier.fillMaxWidth().height(52.dp),
                colors = ButtonDefaults.outlinedButtonColors(contentColor = Color.White)
            ) {
                Text("119 신고", fontWeight = FontWeight.Bold, fontSize = 16.sp)
            }
        }
    }
}

@Composable
private fun InfoPanel(info: EmergencyInfo) {
    SectionCard(backgroundColor = Color.Black.copy(alpha = 0.22f)) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Column {
                Text(if (info.kind == EmergencyKind.FIRE) "감지 유형" else "가스 종류", color = Color.White.copy(alpha = 0.8f), fontSize = 11.sp)
                Text(info.gasType, color = Color.White, fontSize = 17.sp, fontWeight = FontWeight.Bold)
            }
            Column {
                Text("위험 등급", color = Color.White.copy(alpha = 0.8f), fontSize = 11.sp)
                Text("위험", color = Color.White, fontSize = 17.sp, fontWeight = FontWeight.Black)
            }
        }
    }
}

// Note: the pulsing background animation won't play in the static preview pane —
// run on a device/emulator to see it animate, or use "Interactive Mode" in the preview.
@Preview(showBackground = true, name = "비상 경보 전체화면")
@Composable
private fun EmergencyScreenPreview() {
    EmergencyScreen(info = PreviewData.emergencyUiState.emergency, onAcknowledge = {}, onCall119 = {})
}
