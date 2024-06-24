package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ActionCommand;

@Config
@Autonomous(name = "BlueShortTest", group = "Autonomous")
public class BlueShort extends CommandOpMode {
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;

    @Override
    public void initialize() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(-90)));

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        OdometryLiftSubsystem odometryLift = new OdometryLiftSubsystem(this);

        odometryLift.lower();

        register(intake, outtake, odometryLift);

        intake.setClaw(IntakeSubsystem.ClawState.CLOSED);
        outtake.setStopperState(OuttakeSubsystem.BlockerState.ONE_PIXEL);

        Action startTrajectory = drive.actionBuilder(new Pose2d(11.8, 61.7, Math.toRadians(-90)))
                .lineToY(37)
                .build();

        Action boardTrajectory = drive.actionBuilder(new Pose2d(11.8, 37, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(49, Math.toRadians(170))
                .strafeToConstantHeading(new Vector2d(49, 30))
                .build();


        schedule(
                new SequentialCommandGroup(
                        new ActionCommand(startTrajectory),
                        new InstantCommand(() -> intake.setLift(IntakeSubsystem.LiftState.LOWERED)),
                        new WaitCommand(500),
                        new InstantCommand(() -> intake.toggleClaw()),
                        new InstantCommand(() -> intake.setLift(IntakeSubsystem.LiftState.STACK)),
                        new ActionCommand(boardTrajectory),
                        new InstantCommand(() -> outtake.toggleSpike()),
                        new WaitCommand(750),
                        new InstantCommand(() -> outtake.setStopperState(OuttakeSubsystem.BlockerState.FREE)),
                        new WaitCommand(450),
                        new InstantCommand(() -> outtake.toggleSpike()),
                        new InstantCommand(() -> outtake.setSlidesPosition(0))
                )
        );
    }
}
