package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Subsystem for the drive mechanism of the robot.
 */
public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private DoubleSupplier forward, strafe, rotation;
    private final List<Motor> motors;

    /**
     * Create a new instance of the drive subsystem.
     *
     * @param hardwareMap The {@link HardwareMap} to use.
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        motors = new ArrayList<>();
        motors.add(new Motor(hardwareMap, "leftFront"));
        motors.add(new Motor(hardwareMap, "rightFront"));
        motors.add(new Motor(hardwareMap, "leftBack"));
        motors.add(new Motor(hardwareMap, "rightBack"));

        drive = new MecanumDrive(
                motors.get(0),
                motors.get(1),
                motors.get(2),
                motors.get(3)
        );

        motors.forEach(motor -> motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE));
    }

    /**
     * Set drive axes for the drive mechanism.
     * <p>
     * Drive axes have to be {@link DoubleSupplier}.
     *
     * @param forward  Forward axis
     * @param strafe   Strafe axis
     * @param rotation Rotation axis
     */
    public void setAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    @Override
    public void periodic() {
        drive.driveRobotCentric(
                strafe.getAsDouble(),
                forward.getAsDouble(),
                rotation.getAsDouble(),
                false
        );
    }

    public void setMaxSpeed(double speed) {
        drive.setMaxSpeed(speed);
    }
}
