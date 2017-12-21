package org.firstinspires.ftc.teamcode.hardware.gamepads.configs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.gamepads.GamepadConfig;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class PrometheusConfigOne extends GamepadConfig {

    public PrometheusConfigOne(Gamepad gamepad1, Gamepad gamepad2) {
        super(gamepad1, gamepad2);
    }

    //Once receive states check if contains a specific step or not then execute. mb use reflection to simply call the method isntead of 1mil methods?
    @Override
    /**
     * Maps gamepad controls to robot movements
     */
    public ArrayList<RobotStates> getStates() {
        ArrayList<RobotStates> currentStates = new ArrayList<>();

        if (getGamepad1().isLeftStickUp()) {
            currentStates.add(RobotStates.DRIVE_FORWARD); //make sure theres a way to remove after setting it
        }

        if (getGamepad1().isLeftStickDown()) {
            currentStates.add(RobotStates.DRIVE_REVERSE);
        }

        if (getGamepad1().isLeftStickLeft()) {
            currentStates.add(RobotStates.DRIVE_LEFT);
        }

        if (getGamepad1().isLeftStickRight()) {
            currentStates.add(RobotStates.DRIVE_RIGHT);
        }

        if (getGamepad1().isLeftStickIdle()) {
            currentStates.add(RobotStates.DRIVE_BRAKE);
        }

        if (getGamepad1().isLeftTriggerPressed()) {
            currentStates.add(RobotStates.ROTATE_COUNTERCLOCKWISE);
        } else {
            currentStates.add(RobotStates.ROTATE_BRAKE);
        }

        if (getGamepad1().isRightTriggerPressed()) {
            currentStates.add(RobotStates.ROTATE_CLOCKWISE);
        } else {
            currentStates.add(RobotStates.ROTATE_BRAKE);
        }

        if (getGamepad1().getGamepad().left_bumper) {
            currentStates.add(RobotStates.SLOW);
        }

        if (getGamepad2().getGamepad().dpad_up) {
            currentStates.add(RobotStates.RAISE_LIFT);
        }

        if (getGamepad2().getGamepad().dpad_down) {
            currentStates.add(RobotStates.LOWER_LIFT);
        }

        if (getGamepad2().getGamepad().y) {
            currentStates.add(RobotStates.INCREMENT_LIFT);
        }

        if (getGamepad2().getGamepad().a) {
            currentStates.add(RobotStates.DECREMENT_LIFT);
        }

        if (getGamepad2().isLeftStickLeft()) {
            currentStates.add(RobotStates.OPEN_LOWER_GLYPH);
        }

        if (getGamepad2().isLeftStickRight()) {
            currentStates.add(RobotStates.CLOSE_LOWER_GLYPH);
        }

        if (getGamepad2().isRightStickLeft()) {
            currentStates.add(RobotStates.OPEN_UPPER_GLYPH);
        }

        if (getGamepad2().isRightStickRight()) {
            currentStates.add(RobotStates.CLOSE_UPPER_GLYPH);
        }

        return currentStates;
    }
}
