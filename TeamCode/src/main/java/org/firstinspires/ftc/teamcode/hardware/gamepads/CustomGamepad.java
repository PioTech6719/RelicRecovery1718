package org.firstinspires.ftc.teamcode.hardware.gamepads;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Constants;

public class CustomGamepad {

    private final double GAMEPAD_THRESHOLD = Constants.GAMEPAD_THRESHOLD;
    private Gamepad gamepad = null;

    public CustomGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    private JoystickDirection getCurrentJoystickDirection(Joystick joystick) {
        switch (joystick) {
            case LEFT:
                if (-gamepad.left_stick_y > GAMEPAD_THRESHOLD &&
                        gamepad.left_stick_x < GAMEPAD_THRESHOLD &&
                        gamepad.left_stick_x > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.UP;
                } else if (-gamepad.left_stick_y < -GAMEPAD_THRESHOLD &&
                        gamepad.left_stick_x < GAMEPAD_THRESHOLD &&
                        gamepad.left_stick_x > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.DOWN;
                } else if (gamepad.left_stick_x < -GAMEPAD_THRESHOLD &&
                        -gamepad.left_stick_y < GAMEPAD_THRESHOLD &&
                        -gamepad.left_stick_y > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.LEFT;
                } else if (gamepad.left_stick_x > GAMEPAD_THRESHOLD &&
                        -gamepad.left_stick_y < GAMEPAD_THRESHOLD &&
                        -gamepad.left_stick_y > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.RIGHT;
                }
                break;
            case RIGHT:
                if (-gamepad.right_stick_y > GAMEPAD_THRESHOLD &&
                        gamepad.right_stick_x < GAMEPAD_THRESHOLD &&
                        gamepad.right_stick_x > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.UP;
                } else if (-gamepad.right_stick_y < -GAMEPAD_THRESHOLD &&
                        gamepad.right_stick_x < GAMEPAD_THRESHOLD &&
                        gamepad.right_stick_x > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.DOWN;
                } else if (gamepad.right_stick_x < -GAMEPAD_THRESHOLD &&
                        -gamepad.right_stick_y < GAMEPAD_THRESHOLD &&
                        -gamepad.right_stick_y > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.LEFT;
                } else if (gamepad.right_stick_x > GAMEPAD_THRESHOLD &&
                        -gamepad.right_stick_y < GAMEPAD_THRESHOLD &&
                        -gamepad.right_stick_y > -GAMEPAD_THRESHOLD) {
                    return JoystickDirection.RIGHT;
                }
                break;
        }

        return JoystickDirection.NONE;
    }

    private boolean getCurrentTriggerState(Trigger trigger) {
        switch (trigger) {
            case LEFT:
                if (gamepad.left_trigger > GAMEPAD_THRESHOLD) {
                    return true;
                }
                break;
            case RIGHT:
                if (gamepad.right_trigger > GAMEPAD_THRESHOLD) {
                    return true;
                }
                break;
        }

        return false;
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    public boolean isLeftStickUp() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.UP;
    }

    public boolean isLeftStickDown() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.DOWN;
    }

    public boolean isLeftStickLeft() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.LEFT;
    }

    public boolean isLeftStickRight() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.RIGHT;
    }

    public boolean isLeftStickIdle() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.NONE;
    }

    public boolean isRightStickUp() {
        return getCurrentJoystickDirection(Joystick.RIGHT) == JoystickDirection.UP;
    }

    public boolean isRightStickDown() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.DOWN;
    }

    public boolean isRightStickLeft() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.LEFT;
    }

    public boolean isRightStickRight() {
        return getCurrentJoystickDirection(Joystick.LEFT) == JoystickDirection.RIGHT;
    }

    public boolean isRightStickIdle() {
        return getCurrentJoystickDirection(Joystick.RIGHT) == JoystickDirection.NONE;
    }

    public boolean isLeftTriggerPressed() {
        return getCurrentTriggerState(Trigger.LEFT);
    }

    public boolean isRightTriggerPressed() {
        return getCurrentTriggerState(Trigger.RIGHT);
    }

    public boolean isClicked(ButtonType buttonType) {
        return new Button(gamepad, buttonType).isClicked();
    }

    public boolean isPressed(ButtonType buttonType) {
        return new Button(gamepad, buttonType).isPressed();
    }

    public boolean isUpdated(ButtonType buttonType) {
        return new Button(gamepad, buttonType).isUpdated();
    }

    private enum JoystickDirection {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        NONE
    }

    private enum Joystick {
        LEFT,
        RIGHT
    }

    private enum Trigger {
        LEFT,
        RIGHT
    }
}
