package com.gasrobot.monitor.ui.viewmodel

import android.content.Context
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.viewModelScope
import com.gasrobot.monitor.data.local.SessionPrefs
import com.gasrobot.monitor.data.model.AuthRequest
import com.gasrobot.monitor.data.model.AuthUser
import com.gasrobot.monitor.data.network.ApiService
import com.gasrobot.monitor.data.network.NetworkModule
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import retrofit2.HttpException

sealed interface AuthUiState {
    data object Idle : AuthUiState
    data object Loading : AuthUiState
    data class Error(val message: String) : AuthUiState
}

/**
 * Talks to the real /api/auth/register and /api/auth/login endpoints. There's no token to
 * manage — success just means "here's the user row", which gets written to SessionPrefs so
 * MainActivity can skip the login screen on the next launch.
 */
class AuthViewModel(
    private val appContext: Context,
    private val api: ApiService = NetworkModule.apiService
) : ViewModel() {

    private val _state = MutableStateFlow<AuthUiState>(AuthUiState.Idle)
    val state: StateFlow<AuthUiState> = _state.asStateFlow()

    private val _loggedInUser = MutableStateFlow(SessionPrefs.currentUser(appContext))
    val loggedInUser: StateFlow<AuthUser?> = _loggedInUser.asStateFlow()

    fun login(username: String, password: String) = submit(
        request = { api.login(AuthRequest(username.trim(), password, role = "staff")) },
        wrongCredentialsMessage = "아이디 또는 비밀번호가 틀렸습니다"
    )

    fun register(username: String, password: String) = submit(
        request = { api.register(AuthRequest(username.trim(), password, role = "staff")) },
        wrongCredentialsMessage = "이미 사용 중인 아이디입니다"
    )

    fun logout() {
        SessionPrefs.clear(appContext)
        _loggedInUser.value = null
    }

    private fun submit(request: suspend () -> AuthUser, wrongCredentialsMessage: String) {
        viewModelScope.launch {
            _state.value = AuthUiState.Loading
            runCatching { request() }
                .onSuccess { user ->
                    SessionPrefs.save(appContext, user)
                    _loggedInUser.value = user
                    _state.value = AuthUiState.Idle
                }
                .onFailure { e ->
                    val message = when {
                        e is HttpException && (e.code() == 400 || e.code() == 401) -> wrongCredentialsMessage
                        e is HttpException -> "서버 오류 (HTTP ${e.code()})"
                        else -> "서버에 연결할 수 없습니다 — 네트워크를 확인해주세요"
                    }
                    _state.value = AuthUiState.Error(message)
                }
        }
    }
}

class AuthViewModelFactory(private val context: Context) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        @Suppress("UNCHECKED_CAST")
        return AuthViewModel(context.applicationContext) as T
    }
}
