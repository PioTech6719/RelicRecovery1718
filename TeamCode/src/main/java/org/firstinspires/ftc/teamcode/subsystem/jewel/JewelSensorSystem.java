package org.firstinspires.ftc.teamcode.subsystem.jewel;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotServo;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.subsystem.sensors.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.terminators.Status;
import org.firstinspires.ftc.teamcode.utils.PioTimer;
import org.firstinspires.ftc.teamcode.utils.Range;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;


public class JewelSensorSystem extends Subsystem {

    private ColorSensorSystem jewelColorSensorSystem;
    private RobotServo jewelArmServo;
    private boolean initJewelArm;

    private double BLUE_HUE_MIN = 150, BLUE_HUE_MAX = 270;
    private double RED_ONE_HUE_MIN = 0, RED_ONE_HUE_MAX = 30;
    private double RED_TWO_HUE_MIN = 300, RED_TWO_HUE_MAX = 360;

    public JewelSensorSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
        jewelColorSensorSystem = new ColorSensorSystem(getRobot(), getGameMode());
        jewelArmServo = getRobot().addServo(PrometheusConstants.JEWEL_SERVO);
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
        if (initJewelArm) {
            initializeServo();
        }
    }

    @Override
    public void stop() {

    }

    public void initializeServo() {
        jewelArmServo.setPosition(0.5);
        initJewelArm = true;
    }

    public void resetServo() {
        jewelArmServo.setPosition(0.5);

        PioTimer pioTimer = new PioTimer(ElapsedTime.Resolution.SECONDS, 1, 0); //useful or no?
        while (!pioTimer.isFinished() && !Status.isStopRequested()) {
        }
    }

    public void dropServo() {
        jewelArmServo.setPosition(1.0);

        PioTimer pioTimer = new PioTimer(ElapsedTime.Resolution.SECONDS, 1, 0);
        while (!pioTimer.isFinished() && !Status.isStopRequested()) {
        }
    }

    public Jewel getJewel() {
        if (Range.inRange(jewelColorSensorSystem.getHue(), BLUE_HUE_MIN, BLUE_HUE_MAX, true)) {
            return Jewel.BLUE;
        } else if (Range.inRange(jewelColorSensorSystem.getHue(), RED_ONE_HUE_MIN, RED_ONE_HUE_MAX, true)) {
            return Jewel.RED;
        } else if (Range.inRange(jewelColorSensorSystem.getHue(), RED_TWO_HUE_MIN, RED_TWO_HUE_MAX, true)) {
            return Jewel.RED;
        }

        return Jewel.NONE;
    }

    public ColorSensorSystem getJewelColorSensorSystem() {
        return jewelColorSensorSystem;
    }
}
