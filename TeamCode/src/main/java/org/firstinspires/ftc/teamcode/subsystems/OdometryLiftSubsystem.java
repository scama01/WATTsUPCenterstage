package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.InterpolatedServo;

@Config
public class OdometryLiftSubsystem extends SubsystemBase {
    public static double PARALLEL_UP = 0, PARALLEL_DOWN = 1;
    public static double PERPENDICULAR_UP = 0, PERPENDICULAR_DOWN = 1;
    private final InterpolatedServo left, right, perpendicular;


    public OdometryLiftSubsystem(final OpMode opMode) {
        left = new InterpolatedServo(new SimpleServo(opMode.hardwareMap, "odo_left", 0, 180));
        right = new InterpolatedServo(new SimpleServo(opMode.hardwareMap, "odo_right", 0, 180));
        perpendicular = new InterpolatedServo(new SimpleServo(opMode.hardwareMap, "odo_back", 0, 1800));

        left.setInverted(true);

        left.generatePositions(new Pair<>(0.0, 90.0), new Pair<>(1.0, 128.0));
        right.generatePositions(new Pair<>(0.0, 82.0), new Pair<>(1.0, 119.0));
        perpendicular.generatePositions(new Pair<>(0.0, 25.0), new Pair<>(1.0, 125.0));

        if (opMode.getClass().isAnnotationPresent(TeleOp.class)) raise();
        else lower();
    }

    public void raise() {
        left.setToPosition(PARALLEL_UP);
        right.setToPosition(PARALLEL_UP);
        perpendicular.setToPosition(PERPENDICULAR_UP);
    }

    public void lower() {
        left.setToPosition(PARALLEL_DOWN);
        right.setToPosition(PARALLEL_DOWN);
        perpendicular.setToPosition(PERPENDICULAR_DOWN);
    }
}
