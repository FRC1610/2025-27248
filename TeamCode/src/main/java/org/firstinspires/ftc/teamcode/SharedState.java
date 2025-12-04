package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.subsystems.ReadObelisk;

/**
 * Persists match-long values that need to survive across autonomous and tele-op OpModes
 * without requiring a full app restart. Backed by SharedPreferences so values remain
 * available across init/stop transitions.
 */
public final class SharedState {

    private static final String PREF_FILE = "teamcode_shared_state";
    private static final String KEY_OBELISK_PATTERN = "obelisk_pattern";

    private SharedState() {
        // Utility class
    }

    private static SharedPreferences prefs() {
        return AppUtil.getDefContext().getSharedPreferences(PREF_FILE, Context.MODE_PRIVATE);
    }

    public static void saveObeliskPattern(ReadObelisk.ObeliskPattern pattern) {
        if (pattern == null) {
            return;
        }
        ReadObelisk.ArtifactColor[] colors = pattern.getColors();
        StringBuilder encoded = new StringBuilder(3);
        for (ReadObelisk.ArtifactColor color : colors) {
            encoded.append(color == ReadObelisk.ArtifactColor.GREEN ? 'G' : 'P');
        }
        prefs().edit().putString(KEY_OBELISK_PATTERN, encoded.toString()).apply();
    }

    public static ReadObelisk.ObeliskPattern loadObeliskPattern() {
        String stored = prefs().getString(KEY_OBELISK_PATTERN, null);
        if (stored == null || stored.length() != 3) {
            return null;
        }

        ReadObelisk.ArtifactColor[] colors = new ReadObelisk.ArtifactColor[3];
        for (int i = 0; i < 3; i++) {
            char code = Character.toUpperCase(stored.charAt(i));
            colors[i] = code == 'G' ? ReadObelisk.ArtifactColor.GREEN : ReadObelisk.ArtifactColor.PURPLE;
        }

        return new ReadObelisk.ObeliskPattern(colors[0], colors[1], colors[2]);
    }
}
