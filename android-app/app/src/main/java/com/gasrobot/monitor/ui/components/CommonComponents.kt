package com.gasrobot.monitor.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.gasrobot.monitor.ui.theme.AppColors

/** Rounded, colored container re-used across every screen (hero status card, recommendation
 *  box, etc.) — the one shared piece worth keeping here. */
@Composable
fun SectionCard(
    modifier: Modifier = Modifier,
    backgroundColor: Color = Color.White,
    shape: Shape = MaterialTheme.shapes.large,
    padding: Int = 16,
    content: @Composable ColumnScope.() -> Unit
) {
    Column(
        modifier
            .fillMaxWidth()
            .background(backgroundColor, shape)
            .padding(padding.dp),
        content = content
    )
}

@Preview(showBackground = true)
@Composable
private fun CommonComponentsPreview() {
    MaterialTheme {
        SectionCard(backgroundColor = AppColors.SafeBg) {
            Text("SectionCard 예시", fontWeight = FontWeight.Bold)
        }
    }
}
