package org.firstinspires.ftc.teamcode.hardware.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.glyph.GlyphSystem;
import org.firstinspires.ftc.teamcode.subsystem.jewel.JewelSensorSystem;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftSystem;
import org.firstinspires.ftc.teamcode.subsystem.sensors.DistanceSensorSystem;
import org.firstinspires.ftc.teamcode.subsystem.sensors.LightSensorSystem;
import org.firstinspires.ftc.teamcode.subsystem.sensors.camera.PhoneCameraSystem;
import org.firstinspires.ftc.teamcode.subsystem.sensors.gyro.GyroSensorSystem;

public class Prometheus extends Robot {

    private DriveSystem driveSystem = null;
    private GlyphSystem glyphSystem = null;
    private LiftSystem liftSystem = null;
    private JewelSensorSystem jewelSensorSystem = null;
    private DistanceSensorSystem distanceSensorSystem = null;
    private GyroSensorSystem gyroSensorSystem = null;
    private LightSensorSystem lightSensorSystem = null;
    private PhoneCameraSystem phoneCameraSystem = null;

    public Prometheus(OpMode opMode) {
        super(opMode);
    }

    private boolean connectDevices(GameMode gameMode) {
        driveSystem = (DriveSystem) addSubsystem(new DriveSystem(this, gameMode));
        glyphSystem = (GlyphSystem) addSubsystem(new GlyphSystem(this, gameMode));
        liftSystem = (LiftSystem) addSubsystem(new LiftSystem(this, gameMode));

        if (gameMode.equals(GameMode.AUTO)) {
            jewelSensorSystem = (JewelSensorSystem) addSubsystem(new JewelSensorSystem(this, gameMode));
//            distanceSensorSystem = (DistanceSensorSystem) addSubsystem(new DistanceSensorSystem(this, gameMode));
            gyroSensorSystem = (GyroSensorSystem) addSubsystem(new GyroSensorSystem(this, gameMode));
//            lightSensorSystem = (LightSensorSystem) addSubsystem(new LightSensorSystem(this));
//            phoneCameraSystem = (PhoneCameraSystem) addSubsystem(new PhoneCameraSystem(this));
        }

        //TODO: Return whether or not all devices are connected or some are null
        return true;
    }

    @Override
    public void init(GameMode gameMode) {
        connectDevices(gameMode);
    }

    //TODO: Catch exception and prevent error out if motors are null
}
