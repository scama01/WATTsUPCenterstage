package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedServo;

/**
 * Subsystem for the intake of the robot.
 */
@Config
public class IntakeSubsystem extends SubsystemBase {
    /*
        public static double OPEN_ANGLE = 47.5, RAISED_OPEN_ANGLE = 70, CLOSED_ANGLE = 90;
        public static double LOWER_LIFT = 165.0, RAISE_LIFT = 8.0, STACK_LIFT = 146.0;
    */

    public static double LOWER_LIFT = 165.0, RAISE_LIFT = 8.0, STACK_LIFT = 146.0;

    private double getLiftAngle(LiftState state) {
        switch (state) {
            case STACK:
                return STACK_LIFT;
            case RAISED:
                return RAISE_LIFT;
            case LOWERED:
                return LOWER_LIFT;
            default:
                return 0.0;
        }
    }

    public static double OPEN_ANGLE = 47.5, RAISED_ANGLE = 70, CLOSED_ANGLE = 90;

    private double getClawAngle(ClawState state) {
        switch (state) {
            case OPEN:
                return OPEN_ANGLE;
            case RAISED:
                return RAISED_ANGLE;
            case CLOSED:
                return CLOSED_ANGLE;
            default:
                return 0.0;
        }
    }


    private final ServoEx clawServo;
    private final InterpolatedServo left, right;

    public enum ClawState {
        OPEN,
        /**
         * Opened state when claw is raised.
         */
        RAISED,
        CLOSED
    }

    public enum LiftState {
        LOWERED,
        STACK,
        RAISED,
    }

    private ClawState clawState = null;

    private LiftState liftState = null;

    /**
     * Create a new instance of the intake subsystem.
     *
     * @param hw Hardware Map of the robot.
     */
    public IntakeSubsystem(HardwareMap hw) {
        clawServo = new SimpleServo(hw, "claw", 0, 220);
        clawServo.setInverted(true);

        left = new InterpolatedServo(new SimpleServo(hw, "v4b_left", 0, 220));
        right = new InterpolatedServo(new SimpleServo(hw, "v4b_right", 0, 220));

        left.setInverted(false);
        right.setInverted(true);

        left.generatePositions(
                new Pair<>(0.0, 15.0),
                new Pair<>(90.0, 112.5),
                new Pair<>(180.0, 210.0)
        );
        right.generatePositions(
                new Pair<>(0.0, 6.5),
                new Pair<>(90.0, 101.0),
                new Pair<>(180.0, 199.0)
        );

        setClaw(ClawState.CLOSED);
        setLift(LiftState.RAISED);
    }

    /**
     * Set lift position to a certain {@link LiftState}.
     *
     * @param state The {@link LiftState} to set the position to.
     */
    public void setLift(LiftState state) {
        if (state == liftState) {
            return;
        }

        if (clawState == ClawState.OPEN && state == LiftState.RAISED) {
            setClaw(ClawState.CLOSED);
        }
        left.setToPosition(getLiftAngle(state));
        right.setToPosition(getLiftAngle(state));

        liftState = state;
    }

    /**
     * Toggles the lift position.
     */
    public void toggleLift() {
        switch (liftState) {
            case RAISED:
                setLift(LiftState.STACK);
                break;

            case LOWERED:
                setLift(LiftState.RAISED);
                break;

            case STACK:
                setLift(LiftState.LOWERED);
                break;
        }
    }

    /**
     * Adjusts lift position by a certain amount.
     *
     * @param adjustment Amount to adjust.
     */
    public void adjustLift(double adjustment) {
        double newLift = MathUtils.clamp(left.getCurrentPosition() + adjustment, 0.0, 180.00);

        left.setToPosition(newLift);
        right.setToPosition(newLift);
    }

    /**
     * Get current lift state.
     *
     * @return Current lift state as a {@link LiftState}.
     */
    public LiftState getLiftState() {
        return liftState;
    }

    /**
     * Toggle claw position.
     */
    public void toggleClaw() {
        switch (clawState) {
            case CLOSED:
                if (liftState == LiftState.RAISED) {
                    setClaw(ClawState.RAISED);
                } else {
                    setClaw(ClawState.OPEN);
                }
                break;
            case OPEN:
                setClaw(ClawState.CLOSED);
                break;
            case RAISED:
                if (liftState != LiftState.RAISED) {
                    setClaw(ClawState.OPEN);
                } else {
                    setClaw(ClawState.CLOSED);
                }
        }
    }

    /**
     * Set the claw position to a certain {@link ClawState}.
     *
     * @param state The {@link ClawState} to set the position to.
     */
    public void setClaw(ClawState state) {
        if (state == clawState) {
            return;
        }

        clawServo.turnToAngle(getClawAngle(state));
        clawState = state;
    }

    /**
     * Get current claw state.
     *
     * @return Current claw state as a {@link ClawState}.
     */
    public ClawState getClawState() {
        return clawState;
    }
}
