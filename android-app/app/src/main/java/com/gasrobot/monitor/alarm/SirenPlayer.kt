package com.gasrobot.monitor.alarm

import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioTrack
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.math.PI
import kotlin.math.sin

/**
 * Generates a two-tone rising/falling siren sweep in code (no bundled audio asset, no
 * recording of any real disaster-alert siren — this is a synthesized approximation meant
 * to be loud and unmistakable, not a reproduction of a specific broadcast sound).
 *
 * Loops until stop() is called. Runs on its own coroutine so it can be started/stopped
 * independently of the UI lifecycle (e.g. from the foreground service while the app is
 * backgrounded, or from EmergencyActivity while it's in the foreground).
 */
class SirenPlayer {
    private val sampleRate = 44100
    private var job: Job? = null
    private var track: AudioTrack? = null

    fun start(scope: CoroutineScope) {
        if (job?.isActive == true) return
        job = scope.launch(Dispatchers.Default) {
            val minBuf = AudioTrack.getMinBufferSize(
                sampleRate, AudioFormat.CHANNEL_OUT_MONO, AudioFormat.ENCODING_PCM_16BIT
            )
            val t = AudioTrack.Builder()
                .setAudioAttributes(
                    AudioAttributes.Builder()
                        // NOTIFICATION_RINGTONE (not ALARM) so the siren follows the phone's
                        // 알림/벨소리 volume slider — most people carry media volume low/muted
                        // day-to-day but keep ringtone/notification volume audible.
                        .setUsage(AudioAttributes.USAGE_NOTIFICATION_RINGTONE)
                        .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
                        .build()
                )
                .setAudioFormat(
                    AudioFormat.Builder()
                        .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                        .setSampleRate(sampleRate)
                        .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                        .build()
                )
                .setBufferSizeInBytes(minBuf * 2)
                .setTransferMode(AudioTrack.MODE_STREAM)
                .build()
            track = t
            t.play()

            // One sweep cycle: 0.9s rising 600Hz->1200Hz, 0.9s falling back down.
            val sweepSeconds = 0.9
            val samplesPerSweep = (sampleRate * sweepSeconds).toInt()
            val buffer = ShortArray(samplesPerSweep)
            var phase = 0.0

            while (isActive) {
                fillSweep(buffer, freqStart = 600.0, freqEnd = 1200.0, startPhase = phase).let { phase = it }
                t.write(buffer, 0, buffer.size)
                fillSweep(buffer, freqStart = 1200.0, freqEnd = 600.0, startPhase = phase).let { phase = it }
                t.write(buffer, 0, buffer.size)
            }
        }
    }

    private fun fillSweep(buffer: ShortArray, freqStart: Double, freqEnd: Double, startPhase: Double): Double {
        var phase = startPhase
        val n = buffer.size
        for (i in 0 until n) {
            val t = i.toDouble() / n
            val freq = freqStart + (freqEnd - freqStart) * t
            phase += 2 * PI * freq / sampleRate
            val amplitude = 0.85 * Short.MAX_VALUE
            buffer[i] = (sin(phase) * amplitude).toInt().toShort()
        }
        return phase % (2 * PI)
    }

    fun stop() {
        job?.cancel()
        job = null
        track?.runCatching {
            stop()
            release()
        }
        track = null
    }
}
