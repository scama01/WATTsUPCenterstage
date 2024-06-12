package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedServo;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static int[] slidesPositions = {0, 400, 700, 1000, 1250};

    public static double SPIKE_RAISED_ANGLE = 170, SPIKE_LOWERED_ANGLE = 0;

    public static double RAISE_OFFSET = 35;

    public enum BlockerState {
        TWO_PIXELS(108, 135),
        ONE_PIXEL(108, 90),
        FREE(60, 90);

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

    private final InterpolatedServo leftLift, rightLift;

    private BlockerState blockerState = BlockerState.FREE;

    private SpikeState spikeState = SpikeState.RAISED;
    private boolean spikeRaised = false;

    private BooleanSupplier safeToMove = () -> true;

    private final ServoEx stopperTop, stopperBottom;

    public OuttakeSubsystem(HardwareMap hw) {
        slides = hw.dcMotor.get("gli_sus");
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);

        leftLift = new InterpolatedServo(new SimpleServo(hw, "depo_left", 0, 220));
        rightLift = new InterpolatedServo(new SimpleServo(hw, "depo_right", 0, 220));

        stopperTop = new SimpleServo(hw, "stopper_top", 0, 300);
        stopperBottom = new SimpleServo(hw, "stopper_bottom", 0, 300);

        leftLift.generatePositions(
                new Pair<>(0.0, 1.50),
                new Pair<>(90.0, 94.0),
                new Pair<>(180.0, 187.0)
        );
        rightLift.generatePositions(
                new Pair<>(0.0, 2.0),
                new Pair<>(90.0, 91.0),
                new Pair<>(180.0, 181.0)
        );

        stopperBottom.setInverted(true);

        rightLift.setInverted(true);

        stopperBottom.turnToAngle(BlockerState.FREE.bottomPos);
        stopperTop.turnToAngle(BlockerState.FREE.topPos);

        toggleSpike();
    }

    @Override
    public void periodic() {
        if (raisingSlides) {
            raisingSlides = false;

            if (blockerState == BlockerState.FREE)
                setStopperState(BlockerState.TWO_PIXELS);

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

    public int getSlidesPosition() {
        return slides.getCurrentPosition();
    }

    public int getSlidesTarget() {
        return slides.getTargetPosition();
    }

    public void setSpikeState(SpikeState state) {
        leftLift.setToPosition(state == SpikeState.RAISED ? SPIKE_RAISED_ANGLE : SPIKE_LOWERED_ANGLE);
        rightLift.setToPosition(state == SpikeState.RAISED ? SPIKE_RAISED_ANGLE : SPIKE_LOWERED_ANGLE);

        spikeState = state;
    }

    public void toggleSpike() {
        if (!safeToMove.getAsBoolean())
            return;

        switch (spikeState) {
            case RAISED:
                setSpikeState(SpikeState.LOWERED);
                break;
            case LOWERED:
                setSpikeState(SpikeState.RAISED);
                break;
        }
    }

    public void toggleSpikeRaise() {
        if (!safeToMove.getAsBoolean() || spikeState != SpikeState.RAISED)
            return;

        spikeRaised = !spikeRaised;

        leftLift.setToPosition(spikeRaised ? SPIKE_RAISED_ANGLE - RAISE_OFFSET : SPIKE_RAISED_ANGLE);
        rightLift.setToPosition(spikeRaised ? SPIKE_RAISED_ANGLE - RAISE_OFFSET : SPIKE_RAISED_ANGLE);
    }

    public SpikeState getSpikeState() {
        return spikeState;
    }

    public double getSpikeAngle() {
        return leftLift.getCurrentPosition();
    }

    public void setStopperState(BlockerState state) {
        stopperBottom.turnToAngle(state.getBlockerPositions().first);
        stopperTop.turnToAngle(state.getBlockerPositions().second);

        blockerState = state;
    }

    public void toggleStopper() {
        switch (blockerState) {
            case TWO_PIXELS:
                setStopperState(BlockerState.ONE_PIXEL);
                break;
            case ONE_PIXEL:
                setStopperState(BlockerState.FREE);
                break;
            case FREE:
                setStopperState(BlockerState.TWO_PIXELS);
                break;
        }
    }

    public BlockerState getStopperState() {
        return blockerState;
    }
}
