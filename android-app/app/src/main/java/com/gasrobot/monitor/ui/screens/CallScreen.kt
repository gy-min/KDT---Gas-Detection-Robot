package com.gasrobot.monitor.ui.screens

import android.content.Intent
import android.net.Uri
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gasrobot.monitor.ui.components.SectionCard
import com.gasrobot.monitor.ui.preview.PreviewData
import com.gasrobot.monitor.ui.viewmodel.MonitoringUiState

@Composable
fun CallScreen(state: MonitoringUiState) {
    val context = LocalContext.current
    Column(
        Modifier.fillMaxSize().background(Color(0xFF101418)).padding(24.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Spacer(Modifier.height(44.dp))
        Text("긴급신고 · 소방/구급", color = Color(0xFF8A95A0), fontSize = 13.sp)
        Text("119", color = Color.White, fontSize = 56.sp, fontWeight = FontWeight.Bold)

        Spacer(Modifier.height(30.dp))
        SectionCard(backgroundColor = Color(0xFF1A2027)) {
            Column(Modifier.fillMaxWidth(), horizontalAlignment = Alignment.CenterHorizontally) {
                Text("위치 정보 자동 전송", color = Color(0xFF8A95A0), fontSize = 11.sp)
                Text(state.emergency.zoneLabel.ifBlank { "구역 정보 없음" }, color = Color.White, fontSize = 14.sp, fontWeight = FontWeight.Bold)
                Text("${state.emergency.gasType} · ${state.emergency.ppm} ppm", color = Color(0xFFD67C76), fontSize = 12.sp)
            }
        }

        Spacer(Modifier.weight(1f))

        Button(
            onClick = { context.startActivity(Intent(Intent.ACTION_DIAL, Uri.parse("tel:119"))) },
            shape = CircleShape,
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2EC26B)),
            modifier = Modifier.size(84.dp)
        ) {}
        Spacer(Modifier.height(12.dp))
        Text("눌러서 전화 걸기", color = Color(0xFF8A95A0), fontSize = 12.sp)
    }
}

@Preview(showBackground = true)
@Composable
private fun CallScreenPreview() {
    MaterialTheme { CallScreen(PreviewData.emergencyUiState) }
}
