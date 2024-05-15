package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PETRICA", group = "Linear OpMode")
public class TeleOp extends CommandOpMode {

    private List<LynxModule> hubs;

    private IntakeSubsystem intake = null;

    private OuttakeSubsystem outtake;
    private final int ADJUST_TICKS = 65;

    private EndgameSubsystem endgame;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.reset();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        outtake = new OuttakeSubsystem(hardwareMap);

        endgame = new EndgameSubsystem(hardwareMap);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        register(chassis, outtake, endgame);

        Trigger rightTrigger = new Trigger(() -> gamepad2.right_trigger > .3)
                .and(new Trigger(() -> outtake.getSpikeState() == OuttakeSubsystem.SpikeState.RAISED));

        // Limits
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setMaxSpeed(0.33))
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
                .whenActive(() -> intake.toggleClaw());
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.toggleLift());

        // Slides
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(outtake::raiseSlidesPosition);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(outtake::lowerSlidesPosition);

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(0));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> outtake.adjustSlidesTicks(-ADJUST_TICKS));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> outtake.adjustSlidesTicks(ADJUST_TICKS));

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(outtake::toggleSpike);
//        driver2.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(outtake::toggleBlockers);
        rightTrigger.toggleWhenActive(
                () -> outtake.setSpikePosition(.925),
                () -> outtake.setSpikePosition(OuttakeSubsystem.HIGH_RIGHT)
        );
        // Endgame
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(endgame::toggleElevator);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        schedule(new RunCommand(() -> {
            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addLine();

            telemetry.addData("Claw State", intake != null ? intake.getClawState() : "Not initialized");
            telemetry.addData("Lift State", intake != null ? intake.getLiftState() : "Not initialized");
            telemetry.addLine();

            telemetry.addData("Elevator State", endgame.getElevatorState());
            telemetry.addData("Elevator Angle", endgame.getElevatorAngle());
            telemetry.addLine();

            telemetry.update();
        }));
    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);

        super.run();

        runtime.reset();

        if (intake != null) {
            return;
        }

        intake = new IntakeSubsystem(hardwareMap);

        outtake.setSafeguard(() -> intake.getLiftState() != IntakeSubsystem.LiftState.RAISED);

        register(intake);
    }
}
