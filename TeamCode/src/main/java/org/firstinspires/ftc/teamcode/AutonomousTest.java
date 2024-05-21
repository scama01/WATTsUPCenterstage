package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.OdometryLiftSubsystem;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TEST PETRICA", group = "Autonomous")
public class AutonomousTest extends CommandOpMode {
    public static final double TRACKWIDTH = 9.75;
    public static final double WHEEL_DIAMETER = 0.68897637795275590551181102362205;
    public static final double CENTER_WHEEL_OFFSET = -5.1181102362204724409448818897638;

    static double TICKS_TO_INCHES;

    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {
        List<Motor> motors = new ArrayList<>();
        motors.add(new Motor(hardwareMap, "leftFront"));
        motors.add(new Motor(hardwareMap, "rightFront"));
        motors.add(new Motor(hardwareMap, "leftBack"));
        motors.add(new Motor(hardwareMap, "rightBack"));

        MecanumDrive chassis = new MecanumDrive(
                motors.get(0),
                motors.get(1),
                motors.get(2),
                motors.get(3)
        );

        motors.forEach(motor -> motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE));

        leftEncoder = new MotorEx(hardwareMap, "leftFront", 8192, 435);
        rightEncoder = new MotorEx(hardwareMap, "rightBack", 8192, 435);
        centerEncoder = new MotorEx(hardwareMap, "strafe_pod", 8192, 435);

        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / leftEncoder.getCPR();

        Translation2d frontLeftLocation =
                new Translation2d(0.381, 0.381);
        Translation2d frontRightLocation =
                new Translation2d(0.381, -0.381);
        Translation2d backLeftLocation =
                new Translation2d(-0.381, 0.381);
        Translation2d backRightLocation =
                new Translation2d(-0.381, -0.381);

        MecanumDriveKinematics kinematics = new MecanumDriveKinematics
                (
                        frontLeftLocation, frontRightLocation,
                        backLeftLocation, backRightLocation
                );

        OdometryLiftSubsystem odometryLift = new OdometryLiftSubsystem(this);

        register(odometryLift);

    }
}
