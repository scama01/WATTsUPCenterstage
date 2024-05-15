package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static int[] slidesPositions = {0, 400, 700, 1000, 1250};

    public static double LOW_LEFT = 0.02, LOW_RIGHT = 0.07;
    public static double HIGH_LEFT = 02.77, HIGH_RIGHT = 0.82;

    public enum BlockerState {
        TWO_PIXELS(108, 135), ONE_PIXEL(108, 90), FREE(60, 90);

        private final double bottomPos, topPos;

        BlockerState(double bottom, double top) {
            bottomPos = bottom;
            topPos = top;
        }

        public Pair<Double, Double> getBlockerPositions() {
            return new Pair<>(bottomPos, topPos);
        }
    }

    public enum SpikeState {
        RAISED,
        LOWERED
    }

    private final DcMotor slides;
    private boolean raisingSlides = false;

    private final ServoEx leftLift, rightLift;

    private BlockerState blockerState = BlockerState.FREE;

    private SpikeState spikeState = SpikeState.RAISED;

    private BooleanSupplier safeToMove = () -> true;

    private final ServoEx stopperTop, stopperBottom;

    public OuttakeSubsystem(HardwareMap hw) {
        slides = hw.dcMotor.get("gli_sus");
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);

        leftLift = new SimpleServo(hw, "depo_left", 0, 220);
        rightLift = new SimpleServo(hw, "depo_right", 0, 220);

        stopperTop = new SimpleServo(hw, "stopper_top", 0, 300);
        stopperBottom = new SimpleServo(hw, "stopper_bottom", 0, 300);

        stopperBottom.setInverted(true);

        rightLift.setInverted(true);

        stopperBottom.turnToAngle(60);
        stopperTop.turnToAngle(90);

        toggleSpike();
    }

    @Override
    public void periodic() {
        if (raisingSlides) {
            raisingSlides = false;

            if (blockerState == BlockerState.FREE)
                setStopperPositions(BlockerState.TWO_PIXELS);

            if (spikeState != SpikeState.RAISED)
                toggleSpike();
        }
    }

    public void setSafeguard(BooleanSupplier safeToMove) {
        this.safeToMove = safeToMove;
    }

    public void setSlidesTicks(int ticks) {
        if (!safeToMove.getAsBoolean())
            return;

        raisingSlides = slides.getTargetPosition() < 100 && ticks > 100;

        slides.setTargetPosition(ticks);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
    }


    public void setSlidesPosition(int position) {
        position = MathUtils.clamp(position, 0, 4);
        setSlidesTicks(slidesPositions[position]);
    }

    public void raiseSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(slidesPositions, ticks);

        setSlidesPosition(current_pos < 0 ? -(current_pos + 1) : (current_pos + 1));
    }

    public void lowerSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(slidesPositions, ticks);

        setSlidesPosition(current_pos < 0 ? -(current_pos + 1) - 1 : (current_pos - 1));
    }

    public void adjustSlidesTicks(int ticks) {
        setSlidesTicks(slides.getTargetPosition() + ticks);
    }

    public void setSpikePosition(double position) {
        leftLift.setPosition(MathUtils.clamp(position - 0.05, 0, 1));
        rightLift.setPosition(MathUtils.clamp(position, 0, 1));

        spikeState = (position >= HIGH_RIGHT) ? SpikeState.RAISED : SpikeState.LOWERED;
    }

    public void toggleSpike() {
        if (!safeToMove.getAsBoolean())
            return;

        switch (spikeState) {
            case RAISED:
                leftLift.setPosition(LOW_LEFT);
                rightLift.setPosition(LOW_RIGHT);
                spikeState = SpikeState.LOWERED;
                break;
            case LOWERED:
                leftLift.setPosition(HIGH_LEFT);
                rightLift.setPosition(HIGH_RIGHT);
                spikeState = SpikeState.RAISED;
                break;
        }
    }

    public SpikeState getSpikeState() {
        return spikeState;
    }

    public void setStopperPositions(BlockerState state) {
        stopperBottom.turnToAngle(state.getBlockerPositions().first);
        stopperTop.turnToAngle(state.getBlockerPositions().second);

        blockerState = state;
    }

    public void toggleStopper() {
        switch (blockerState) {
            case TWO_PIXELS:
                setStopperPositions(BlockerState.ONE_PIXEL);
                break;
            case ONE_PIXEL:
                setStopperPositions(BlockerState.FREE);
                break;
            case FREE:
                setStopperPositions(BlockerState.TWO_PIXELS);
                break;
        }
    }

    public BlockerState getStopperState() {
        return blockerState;
    }
}
