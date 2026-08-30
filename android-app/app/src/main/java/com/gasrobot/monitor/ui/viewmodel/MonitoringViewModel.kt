package com.gasrobot.monitor.ui.viewmodel

import android.content.Context
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.viewModelScope
import com.gasrobot.monitor.alarm.AlarmController
import com.gasrobot.monitor.data.repository.MonitoringRepository
import com.gasrobot.monitor.service.MonitoringForegroundService
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

/** State for the evacuation-route picker on EvacScreen. `Idle` = nothing tapped yet;
 *  `route` in `Loaded` is the ordered node-id list from MonitoringRepository.fetchEvacuationRoute
 *  (ready to render as-is, from `currentLocation` to the exit inclusive); `blockedZones` is
 *  whichever zone ids the server routed around for this run. */
sealed interface EvacRouteState {
    data object Idle : EvacRouteState
    data class Loading(val currentLocation: String) : EvacRouteState
    data class Loaded(val currentLocation: String, val route: List<String>, val blockedZones: List<String> = emptyList()) : EvacRouteState
    data class Failed(val currentLocation: String, val message: String) : EvacRouteState
}

/**
 * MVVM ViewModel layer.
 *  - Model: MonitoringRepository (network/state) + AlarmController (siren/vibration/notification).
 *    Both keep running independently inside MonitoringForegroundService too, so alarms still
 *    fire even with no Activity/ViewModel alive — this ViewModel just gives the UI a clean,
 *    lifecycle-aware window into that same shared state.
 *  - View: the Composables in ui/screens, which only ever read `uiState` and call the
 *    functions below; they never touch MonitoringRepository or AlarmController directly.
 */
class MonitoringViewModel(
    private val repository: MonitoringRepository,
    private val alarmController: AlarmController
) : ViewModel() {

    private val _evacRoute = MutableStateFlow<EvacRouteState>(EvacRouteState.Idle)
    val evacRoute: StateFlow<EvacRouteState> = _evacRoute.asStateFlow()

    val uiState: StateFlow<MonitoringUiState> = combine(
        repository.snapshot, repository.connectionState
    ) { snapshot, connection ->
        MonitoringUiState(
            zones = snapshot.zones,
            robotMarker = snapshot.robotMarker,
            emergency = snapshot.emergency,
            directive = snapshot.directive,
            connectionState = connection
        )
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5_000), MonitoringUiState())

    /** Called when the user taps "상황 확인하기" on the full-screen alarm. */
    fun acknowledgeEmergency() {
        alarmController.stop()
    }

    /** Called when the user taps a node on EvacScreen's map. Fires the request immediately (no
     *  debounce/confirm) since a wrong tap just costs one extra request, and every previous
     *  result is instantly replaced — nothing waits on the old one. */
    fun selectCurrentLocation(location: String) {
        _evacRoute.value = EvacRouteState.Loading(location)
        viewModelScope.launch {
            repository.fetchEvacuationRoute(location)
                .onSuccess { (route, blocked) -> _evacRoute.value = EvacRouteState.Loaded(location, route, blocked) }
                .onFailure { e ->
                    _evacRoute.value = EvacRouteState.Failed(
                        location,
                        e.message ?: "경로를 계산할 수 없습니다"
                    )
                }
        }
    }
}

class MonitoringViewModelFactory(private val context: Context) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        val repository = MonitoringForegroundService.getOrCreateRepository()
        val alarmController = MonitoringForegroundService.getOrCreateAlarmController(context)
        @Suppress("UNCHECKED_CAST")
        return MonitoringViewModel(repository, alarmController) as T
    }
}
