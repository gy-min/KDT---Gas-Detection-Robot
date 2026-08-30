package com.gasrobot.monitor.ui

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.gasrobot.monitor.R
import com.gasrobot.monitor.ui.screens.*
import com.gasrobot.monitor.ui.theme.AppColors
import com.gasrobot.monitor.ui.viewmodel.MonitoringUiState
import com.gasrobot.monitor.ui.viewmodel.MonitoringViewModel
import com.gasrobot.monitor.ui.viewmodel.MonitoringViewModelFactory
import com.gasrobot.monitor.data.model.DirectiveInfo
import com.gasrobot.monitor.ui.viewmodel.AuthViewModel
import com.gasrobot.monitor.ui.viewmodel.AuthViewModelFactory

/**
 * MVVM wiring: the Activity's only job is to create the ViewModel (via the factory) and hand
 * its `uiState` down to plain, ViewModel-agnostic Composable screens. No screen in ui/screens
 * talks to the repository, the service, or the alarm controller directly.
 */
class MainActivity : ComponentActivity() {

    private val requestNotifPermission = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { /* no-op: if denied we just fall back to in-app UI, siren/vibration still work */ }

    // singleTask launchMode means an existing instance can be reused via onNewIntent instead of
    // onCreate — so the "route" extra has to live in state we can update from either callback,
    // not just be read once inside onCreate's setContent.
    private val pendingRoute = mutableStateOf<String?>(null)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        ensureNotificationPermission()
        pendingRoute.value = intent.getStringExtra("route")

        setContent {
            val authViewModel: AuthViewModel = viewModel(factory = AuthViewModelFactory(this))
            val loggedInUser by authViewModel.loggedInUser.collectAsState()
            val authState by authViewModel.state.collectAsState()

            MaterialTheme {
                val user = loggedInUser
                if (user == null) {
                    LoginScreen(
                        state = authState,
                        onLogin = authViewModel::login,
                        onRegister = authViewModel::register
                    )
                } else {
                    val viewModel: MonitoringViewModel = viewModel(factory = MonitoringViewModelFactory(this))
                    val uiState by viewModel.uiState.collectAsState()
                    val startRoute by pendingRoute
                    AppScaffold(viewModel, uiState, startRoute ?: "home", onLogout = authViewModel::logout)
                }
            }
        }
    }

    override fun onNewIntent(intent: android.content.Intent) {
        super.onNewIntent(intent)
        setIntent(intent)
        pendingRoute.value = intent.getStringExtra("route")
    }

    private fun ensureNotificationPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            val granted = ContextCompat.checkSelfPermission(
                this, Manifest.permission.POST_NOTIFICATIONS
            ) == PackageManager.PERMISSION_GRANTED
            if (!granted) requestNotifPermission.launch(Manifest.permission.POST_NOTIFICATIONS)
        }
    }
}

private data class NavItem(val route: String, val label: String, val iconRes: Int)

private val NAV_ITEMS = listOf(
    NavItem("home", "홈", R.drawable.ic_nav_home),
    NavItem("evac", "대피", R.drawable.ic_nav_evac),
    NavItem("call", "119", R.drawable.ic_nav_call),
)

@Composable
private fun AppScaffold(
    viewModel: MonitoringViewModel,
    state: MonitoringUiState,
    startRoute: String = "home",
    onLogout: () -> Unit = {}
) {
    val navController = rememberNavController()
    val backStack by navController.currentBackStackEntryAsState()
    val currentRoute = backStack?.destination?.route ?: "home"
    val emergency = state.emergency.active
    val evacRoute by viewModel.evacRoute.collectAsState()

    // Handles both "app wasn't running yet, launch straight into evac" (first composition)
    // and "app was already open, alarm fired again, jump it to evac now" (startRoute changes
    // while MainActivity is reused via onNewIntent) — NavHost's own startDestination only
    // covers the former.
    LaunchedEffect(startRoute) {
        if (startRoute != "home") {
            navController.navigate(startRoute) { launchSingleTop = true }
        }
    }

    // Pops up over whichever tab the user is currently on — see 관리자 지시사항 popup requirement.
    // rememberSaveable so it survives rotation; tracks the last-dismissed directive id so the
    // same instruction doesn't re-pop every recomposition after the user closes it.
    var dismissedDirectiveId by rememberSaveable { mutableStateOf<String?>(null) }
    val directive = state.directive
    if (directive != null && directive.id != dismissedDirectiveId) {
        DirectiveDialog(directive, onDismiss = { dismissedDirectiveId = directive.id })
    }

    Scaffold(
        containerColor = if (emergency) AppColors.PageBgEmergency else AppColors.PageBg,
        bottomBar = {
            NavigationBar {
                NAV_ITEMS.forEach { item ->
                    val selected = currentRoute == item.route
                    val tint = when {
                        item.route == "call" -> AppColors.DangerIcon
                        selected -> AppColors.Primary
                        else -> AppColors.InkFaint
                    }
                    NavigationBarItem(
                        selected = selected,
                        onClick = {
                            navController.navigate(item.route) {
                                popUpTo(navController.graph.startDestinationId)
                                launchSingleTop = true
                            }
                        },
                        icon = {
                            Image(
                                painter = painterResource(id = item.iconRes),
                                contentDescription = item.label,
                                colorFilter = ColorFilter.tint(tint),
                                modifier = Modifier.size(24.dp)
                            )
                        },
                        label = { Text(item.label) },
                        colors = NavigationBarItemDefaults.colors(
                            selectedTextColor = if (item.route == "call") AppColors.DangerIcon else AppColors.Primary
                        )
                    )
                }
            }
        }
    ) { padding ->
        Box(Modifier.padding(padding).fillMaxSize()) {
            NavHost(navController, startDestination = "home") {
                composable("home") {
                    HomeScreen(state, onLogout = onLogout)
                }
                composable("evac") {
                    EvacScreen(state, evacRoute, onSelectLocation = viewModel::selectCurrentLocation)
                }
                composable("call") { CallScreen(state) }
            }
        }
    }
}

/**
 * Full popup for a new admin directive — AlertDialog already closes on both the confirm button
 * and a tap on the scrim outside the box (onDismissRequest covers both), which is exactly the
 * "확인 버튼이나 빈 공간 눌러서 직접 닫기" behavior asked for.
 */
@Composable
private fun DirectiveDialog(directive: DirectiveInfo, onDismiss: () -> Unit) {
    AlertDialog(
        onDismissRequest = onDismiss,
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("확인") }
        },
        title = { Text("관리자 지시사항") },
        text = { Text(directive.message) }
    )
}
