package org.firstinspires.ftc.teamcode.hardware.gamepads.configs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.gamepads.GamepadConfig;

import java.util.ArrayList;

public class ConfigOne extends GamepadConfig {

    public ConfigOne(Gamepad gamepad1, Gamepad gamepad2) {
        super(gamepad1, gamepad2);
    }

    //Once receive states check if contains a specific step or not then execute. mb use reflection to simply call the method isntead of 1mil methods?
    @Override
    /**
     * Maps gamepad controls to robot movements
     */
    public ArrayList<STATES> getStates() {
        ArrayList<STATES> currentStates = new ArrayList<>();

        if (getGamepad1().isLeftStickUp()) {
            currentStates.add(STATES.DRIVE_FORWARD); //make sure theres a way to remove after setting it
        }

        if (getGamepad1().isLeftStickDown()) {
            currentStates.add(STATES.DRIVE_REVERSE);
        }

        if (getGamepad1().isLeftStickLeft()) {
            currentStates.add(STATES.DRIVE_LEFT);
        }

        if (getGamepad1().isLeftStickRight()) {
            currentStates.add(STATES.DRIVE_RIGHT);
        }

        if (getGamepad1().isLeftStickIdle()) {
            currentStates.add(STATES.DRIVE_BRAKE);
        }

        if (getGamepad1().isLeftTriggerPressed()) {
            currentStates.add(STATES.ROTATE_COUNTERCLOCKWISE);
        } else {
            currentStates.add(STATES.ROTATE_BRAKE);
        }

        if (getGamepad1().isRightTriggerPressed()) {
            currentStates.add(STATES.ROTATE_CLOCKWISE);
        } else {
            currentStates.add(STATES.ROTATE_BRAKE);
        }

        if (getGamepad1().getGamepad().left_bumper) {
            currentStates.add(STATES.SLOW);
        }

        if (getGamepad2().getGamepad().dpad_up) {
            currentStates.add(STATES.RAISE_LIFT);
        }

        if (getGamepad2().getGamepad().dpad_down) {
            currentStates.add(STATES.LOWER_LIFT);
        }

        if (getGamepad2().getGamepad().y) {
            currentStates.add(STATES.INCREMENT_LIFT);
        }

        if (getGamepad2().getGamepad().a) {
            currentStates.add(STATES.DECREMENT_LIFT);
        }

        if (getGamepad2().isLeftStickLeft()) {
            currentStates.add(STATES.OPEN_LOWER_GLYPH);
        }

        if (getGamepad2().isLeftStickRight()) {
            currentStates.add(STATES.CLOSE_LOWER_GLYPH);
        }

        if (getGamepad2().isRightStickLeft()) {
            currentStates.add(STATES.OPEN_UPPER_GLYPH);
        }

        if (getGamepad2().isRightStickRight()) {
            currentStates.add(STATES.CLOSE_UPPER_GLYPH);
        }

        return currentStates;
    }

    //TODO: Where should the executeState() be? In the teleop class? pass it to movement controllers
    public enum STATES implements GamepadConfig.STATES {
        DRIVE_FORWARD,
        DRIVE_REVERSE,
        DRIVE_LEFT,
        DRIVE_RIGHT,
        DRIVE_BRAKE,

        ROTATE_CLOCKWISE,
        ROTATE_COUNTERCLOCKWISE,
        ROTATE_BRAKE,

        SLOW,

        RAISE_LIFT,
        LOWER_LIFT,

        INCREMENT_LIFT,
        DECREMENT_LIFT,

        OPEN_LOWER_GLYPH, //TODO: What if wanted to have the joystick value control the setting?
        CLOSE_LOWER_GLYPH,

        OPEN_UPPER_GLYPH,
        CLOSE_UPPER_GLYPH
    }
}
