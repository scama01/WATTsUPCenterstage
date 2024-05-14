package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static int[] slidesPositions = {0, 400, 700, 1000, 1250};

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

    private final BlockerState blockerState = BlockerState.FREE;

    private BooleanSupplier safeToMove = () -> true;

    public OuttakeSubsystem(HardwareMap hw) {
        slides = hw.dcMotor.get("gli_sus");
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
    }

    @Override
    public void periodic() {
        if (raisingSlides) {
            raisingSlides = false;

//            if (blockerState == Blocker.FREE)
//                setStopperPositions(Blocker.TWO_PIXELS);
//
//            if (spikeState != Spike.RAISED)
//                toggleSpike();
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

    public BlockerState getBlockerState() {
        return blockerState;
    }
}
