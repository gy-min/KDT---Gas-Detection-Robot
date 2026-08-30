package com.gasrobot.monitor.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gasrobot.monitor.ui.theme.AppColors
import com.gasrobot.monitor.ui.viewmodel.AuthUiState

/**
 * Gate screen shown before anything else in the app when there's no saved session
 * (SessionPrefs.currentUser() == null). Employees register once, then just log in on later
 * launches — see AuthViewModel for why there's no token to manage.
 */
@Composable
fun LoginScreen(
    state: AuthUiState,
    onLogin: (username: String, password: String) -> Unit,
    onRegister: (username: String, password: String) -> Unit
) {
    var isRegisterMode by remember { mutableStateOf(false) }
    var username by remember { mutableStateOf("") }
    var password by remember { mutableStateOf("") }

    val loading = state is AuthUiState.Loading

    Column(
        Modifier
            .fillMaxSize()
            .padding(28.dp),
        verticalArrangement = Arrangement.Center
    ) {
        Text("가스탐지로봇 모니터링", fontWeight = FontWeight.Black, fontSize = 24.sp, color = AppColors.Ink)
        Text(
            if (isRegisterMode) "직원 계정 등록" else "직원 로그인",
            fontSize = 14.sp, color = AppColors.InkFaint
        )

        Spacer(Modifier.height(28.dp))

        OutlinedTextField(
            value = username,
            onValueChange = { username = it },
            label = { Text("아이디") },
            singleLine = true,
            enabled = !loading,
            modifier = Modifier.fillMaxWidth()
        )
        Spacer(Modifier.height(10.dp))
        OutlinedTextField(
            value = password,
            onValueChange = { password = it },
            label = { Text("비밀번호") },
            singleLine = true,
            enabled = !loading,
            visualTransformation = PasswordVisualTransformation(),
            modifier = Modifier.fillMaxWidth()
        )

        if (state is AuthUiState.Error) {
            Spacer(Modifier.height(10.dp))
            Text(state.message, color = AppColors.DangerText, fontSize = 12.sp)
        }

        Spacer(Modifier.height(18.dp))

        val canSubmit = username.isNotBlank() && password.isNotBlank() && !loading
        Button(
            onClick = {
                if (isRegisterMode) onRegister(username, password) else onLogin(username, password)
            },
            enabled = canSubmit,
            modifier = Modifier.fillMaxWidth().height(52.dp)
        ) {
            if (loading) {
                CircularProgressIndicator(modifier = Modifier.size(20.dp), strokeWidth = 2.dp, color = MaterialTheme.colorScheme.onPrimary)
            } else {
                Text(if (isRegisterMode) "등록하고 시작하기" else "로그인", fontWeight = FontWeight.Bold)
            }
        }

        Spacer(Modifier.height(12.dp))

        TextButton(
            onClick = { isRegisterMode = !isRegisterMode },
            modifier = Modifier.fillMaxWidth(),
            enabled = !loading
        ) {
            Text(
                if (isRegisterMode) "이미 계정이 있어요 · 로그인으로" else "처음이신가요? · 계정 등록",
                fontSize = 13.sp
            )
        }
    }
}

@androidx.compose.ui.tooling.preview.Preview(showBackground = true)
@Composable
private fun LoginScreenPreview() {
    MaterialTheme { LoginScreen(state = AuthUiState.Idle, onLogin = { _, _ -> }, onRegister = { _, _ -> }) }
}
