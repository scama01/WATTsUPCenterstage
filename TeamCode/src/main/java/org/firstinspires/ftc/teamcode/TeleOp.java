package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PETRICA", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    public static int LIFT_WAIT = 150;

    private List<LynxModule> hubs;

    private IntakeSubsystem intake = null;

    private OuttakeSubsystem outtake;
    private final int ADJUST_TICKS = 65;

    private EndgameSubsystem endgame;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.reset();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        outtake = new OuttakeSubsystem(hardwareMap);

        endgame = new EndgameSubsystem(hardwareMap);

        OdometryLiftSubsystem odometryLift = new OdometryLiftSubsystem(this);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        register(chassis, outtake, endgame, odometryLift);

        Trigger rightTrigger = new Trigger(() -> gamepad2.right_trigger > .3)
                .and(new Trigger(() -> outtake.getSpikeState() == OuttakeSubsystem.SpikeState.RAISED));

        // Limits
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setMaxSpeed(0.33))
                .whenReleased(() -> chassis.setMaxSpeed(1));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> chassis.setMaxSpeed(0.6))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        // Intake
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.adjustLift(5.0));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.adjustLift(-5.0));
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> intake != null))
                .whenActive(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> intake.toggleClaw()),
                                        new WaitCommand(LIFT_WAIT),
                                        new InstantCommand(() -> intake.setLift(IntakeSubsystem.LiftState.RAISED))
                                ),
                                new InstantCommand(() -> intake.toggleClaw()),
                                () -> (intake.getLiftState() == IntakeSubsystem.LiftState.STACK || intake.getLiftState() == IntakeSubsystem.LiftState.LOWERED) &&
                                        intake.getClawState() == IntakeSubsystem.ClawState.OPEN
                        )
                );
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> intake != null))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.toggleLift()),
                        new WaitCommand(LIFT_WAIT),
                        new ConditionalCommand(
                                new InstantCommand(() -> intake.toggleClaw()),
                                new InstantCommand(),
                                () -> (intake.getClawState() == IntakeSubsystem.ClawState.CLOSED || intake.getClawState() == IntakeSubsystem.ClawState.RAISED) &&
                                        intake.getLiftState() == IntakeSubsystem.LiftState.STACK
                        )
                ));

        // Slides
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(outtake::raiseSlidesPosition);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(outtake::lowerSlidesPosition);

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> outtake.toggleSpike()),
                                        new InstantCommand(() -> outtake.setStopperState(OuttakeSubsystem.BlockerState.FREE)),
                                        new InstantCommand(() -> outtake.setSlidesPosition(0))
                                ),
                                new InstantCommand(() -> outtake.setSlidesPosition(0)),
                                () -> outtake.getSpikeState() == OuttakeSubsystem.SpikeState.RAISED
                        )
                );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> outtake.adjustSlidesTicks(-ADJUST_TICKS));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> outtake.adjustSlidesTicks(ADJUST_TICKS));

        // Spike
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(outtake::toggleSpike);
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(outtake::toggleStopper);
        rightTrigger.whenActive(outtake::toggleSpikeRaise);

        // Endgame
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(endgame::toggleElevator);
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(endgame::toggleLauncher);

        telemetry.addData("Status", "Initialized");

        schedule(
                new RunCommand(() -> {
                    telemetry.addData("Status", "Running");
                    telemetry.addLine();

                    telemetry.addData("Claw State", intake != null ? intake.getClawState() : "Not initialized");
                    telemetry.addData("Lift State", intake != null ? intake.getLiftState() : "Not initialized");
                    telemetry.addLine();

                    telemetry.addData("Slides Position", outtake.getSlidesPosition());
                    telemetry.addData("Slides Target", outtake.getSlidesTarget());
                    telemetry.addLine();

                    telemetry.addData("Spike State", outtake.getSpikeState());
                    telemetry.addData("Spike Angle", outtake.getSpikeAngle());
                    telemetry.addData("Stopper State", outtake.getStopperState());
                    telemetry.addLine();

                    telemetry.addData("Elevator State", endgame.getElevatorState());
                    telemetry.addData("Elevator Angle", endgame.getElevatorAngle());
                    telemetry.addLine();

                    telemetry.update();
                })
        );
    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);

        super.run();

        if (intake != null) {
            return;
        }

        intake = new IntakeSubsystem(hardwareMap);

        outtake.setSafeguard(() -> intake.getLiftState() != IntakeSubsystem.LiftState.RAISED);

        register(intake);
    }
}
