package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

/**
 * Subsystem for the drive mechanism of the robot.
 */
public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private DoubleSupplier forward, strafe, rotation;
    private final Motor fL, fR, bL, bR;

    /**
     * Create a new instance of the drive subsystem.
     *
     * @param hardwareMap The {@link HardwareMap} to use.
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        fL = new Motor(hardwareMap, "leftFront");
        fR = new Motor(hardwareMap, "rightFront");
        bL = new Motor(hardwareMap, "leftBack");
        bR = new Motor(hardwareMap, "rightBack");

        drive = new MecanumDrive(
                fL, fR, bL, bR
        );
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

    /**
     * Stop the drive system by stopping all motors.
     */
    public void activateBreaks() {
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive.setMaxSpeed(MathUtils.clamp(Math.abs(0), 0, 1));
    }

    /**
     * Reactivate the drive system by turning back on the motors.
     */
    public void deactivateBreaks() {
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        drive.setMaxSpeed(MathUtils.clamp(Math.abs(1), 0, 1));
    }
}
