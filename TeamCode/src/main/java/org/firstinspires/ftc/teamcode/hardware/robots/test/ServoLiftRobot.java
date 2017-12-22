package org.firstinspires.ftc.teamcode.hardware.robots.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.glyph.GlyphSystem;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftSystem;


public class ServoLiftRobot extends Robot {

    private GlyphSystem glyphSystem = null;
    private LiftSystem liftSystem = null;

    public ServoLiftRobot(OpMode opMode) {
        super(opMode);
    }

    private boolean connectDevices(GameMode gameMode) {
        glyphSystem = (GlyphSystem) addSubsystem(new GlyphSystem(this, gameMode));
        liftSystem = (LiftSystem) addSubsystem(new LiftSystem(this, gameMode));

        //Return whether or not all devices are connected or some are null
        return true;
    }

    @Override
    public void init(GameMode gameMode) {
        connectDevices(gameMode);
    }
}
