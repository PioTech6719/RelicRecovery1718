package org.firstinspires.ftc.teamcode.hardware.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Button {

    private Gamepad gamepad;
    private ButtonType type;
    private boolean active, pressed, previousState;

    public Button(Gamepad gamepad, ButtonType type) {
        this.gamepad = gamepad;
        this.type = type;
        active = false;
        pressed = false;
        previousState = false;
    }

    public boolean isClicked() {
        boolean state = isPressed();
        //if the code does know the button has been pressed and it has been pressed
        if (!pressed && state) {
            //change the pressed status to true to wait for the button to be released
            pressed = true;
            //once the pressed status no longer matches the state the button has been released
        } else if (pressed && !state) {
            //change the active state and forget the pressed state
            active = !active;
            pressed = false;
        }
        //if nothing has changed returned the current status
        return active;
    }

    public boolean isPressed() {
        switch (type) {
            case A:
                if (gamepad.a) return true;
                break;
            case B:
                if (gamepad.b) return true;
                break;
            case X:
                if (gamepad.x) return true;
                break;
            case Y:
                if (gamepad.y) return true;
                break;
            case LEFT_TRIGGER:
                if (gamepad.left_trigger > 0) return true;
                break;
            case RIGHT_TRIGGER:
                if (gamepad.right_trigger > 0) return true;
                break;
            case LEFT_BUMPER:
                if (gamepad.left_bumper) return true;
                break;
            case RIGHT_BUMPER:
                if (gamepad.right_bumper) return true;
                break;
            case D_PAD_DOWN:
                if (gamepad.dpad_down) return true;
                break;
            case D_PAD_LEFT:
                if (gamepad.dpad_left) return true;
                break;
            case D_PAD_RIGHT:
                if (gamepad.dpad_right) return true;
                break;
            case D_PAD_UP:
                if (gamepad.dpad_up) return true;
                break;
        }
        return false;
    }

    public boolean isUpdated() {
        boolean state = isPressed(), returnVal = state != previousState;
        previousState = state;
        return returnVal;
    }

}
