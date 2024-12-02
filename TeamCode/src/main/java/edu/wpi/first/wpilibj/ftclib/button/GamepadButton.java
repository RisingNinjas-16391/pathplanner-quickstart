package edu.wpi.first.wpilibj.ftclib.button;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import androidx.annotation.NonNull;

import edu.wpi.first.wpilibj.ftclib.gamepad.GamepadEx;
import edu.wpi.first.wpilibj.ftclib.gamepad.GamepadKeys;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A {@link Trigger} that gets its state from a {@link GamepadEx}.
 *
 * @author Jackson
 */
public class GamepadButton extends Trigger {
    /**
     * Creates a gamepad button for triggering commands.
     *
     * @param gamepad the gamepad with the buttons
     * @param button the specified buttons
     */
    public GamepadButton(GamepadEx gamepad, @NonNull GamepadKeys.Button button) {
        super(() -> gamepad.getButton(button));
        requireNonNullParam(gamepad, "gamepad", "GamepadButton");
    }
}
