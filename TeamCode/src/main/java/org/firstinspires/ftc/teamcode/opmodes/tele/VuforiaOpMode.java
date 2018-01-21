package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.gamepads.configs.PrometheusConfigOne;
import org.firstinspires.ftc.teamcode.hardware.robots.test.CameraRobot;

@TeleOp(name = "Sensor Test - Vuforia", group = "test")
@Disabled
public class VuforiaOpMode extends BaseTeleOp {

    private CameraRobot cameraRobot = null;
    private PrometheusConfigOne gamepadConfig = null;

    @Override
    public void initializeClass() {
        cameraRobot = new CameraRobot(this);
        gamepadConfig = new PrometheusConfigOne(gamepad1, gamepad2);

        setRobot(cameraRobot);
        setGamepadConfig(gamepadConfig);
    }

    @Override
    public void init() {
        super.init();
//        cameraRobot.().toggleVuforia(true);
        //TODO: Have getSubsytem method in Robot so that drivesystem can use gyro heading to drive straight

    }

    @Override
    public void loop() {
        //Do this in base class to robot variable
        cameraRobot.handleOperations(gamepadConfig.getStates());

        telemetry.update();
    }
}
