package com.gasrobot.monitor.data.network

import com.gasrobot.monitor.data.model.AuthRequest
import com.gasrobot.monitor.data.model.AuthUser
import com.gasrobot.monitor.data.model.EvacuationRouteDto
import com.gasrobot.monitor.data.model.FireEventDto
import com.gasrobot.monitor.data.model.InstructionDto
import com.gasrobot.monitor.data.model.RobotEventDto
import com.gasrobot.monitor.data.model.ZoneReadingDto
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Query

/**
 * Calls the real FastAPI server (main.py) — no invented "/monitoring/snapshot" endpoint;
 * this hits the actual routes and reassembles them into the app's internal models in
 * MonitoringRepository. REST is the bootstrap/fallback path (used on cold start and whenever
 * the WebSocket is down, polled every POLL_INTERVAL_MS); the WebSocket is the primary live path.
 */
interface ApiService {
    @POST("auth/register")
    suspend fun register(@Body request: AuthRequest): AuthUser

    @POST("auth/login")
    suspend fun login(@Body request: AuthRequest): AuthUser

    @GET("fixed-sensor/latest")
    suspend fun getFixedSensorLatest(): List<ZoneReadingDto>

    @GET("robot-events")
    suspend fun getRobotEvents(@Query("limit") limit: Int = 1): List<RobotEventDto>

    @GET("fire-events")
    suspend fun getFireEvents(@Query("limit") limit: Int = 1): List<FireEventDto>

    @GET("instructions")
    suspend fun getInstructions(@Query("limit") limit: Int = 1): List<InstructionDto>

    @GET("evacuation-route")
    suspend fun getEvacuationRoute(@Query("current_location") currentLocation: String): EvacuationRouteDto

    companion object {
        const val POLL_INTERVAL_MS = 5_000L
    }
}
