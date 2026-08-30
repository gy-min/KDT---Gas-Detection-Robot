package com.gasrobot.monitor.data.local

import android.content.Context
import com.gasrobot.monitor.data.model.AuthUser

/**
 * The server (main.py) issues no token/session — login just returns the user row. So "being
 * logged in" on this phone is entirely a local concept: remember the last successful
 * login/register result in SharedPreferences and treat its presence as "still logged in".
 * There's no expiry and no way to invalidate remotely — if that's needed later, this is the
 * one place to add a token field and start sending it as a header.
 */
object SessionPrefs {
    private const val PREFS_NAME = "session_prefs"
    private const val KEY_USER_ID = "user_id"
    private const val KEY_USERNAME = "username"
    private const val KEY_ROLE = "role"

    fun currentUser(context: Context): AuthUser? {
        val prefs = prefs(context)
        val id = prefs.getInt(KEY_USER_ID, -1)
        val username = prefs.getString(KEY_USERNAME, null)
        val role = prefs.getString(KEY_ROLE, null)
        if (id < 0 || username == null || role == null) return null
        return AuthUser(id, username, role)
    }

    fun save(context: Context, user: AuthUser) {
        prefs(context).edit()
            .putInt(KEY_USER_ID, user.id)
            .putString(KEY_USERNAME, user.username)
            .putString(KEY_ROLE, user.role)
            .apply()
    }

    fun clear(context: Context) {
        prefs(context).edit().clear().apply()
    }

    private fun prefs(context: Context) =
        context.applicationContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
}
