package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

public class IndependentMotorGroup implements Iterable<Motor> {

    private final List<Motor> motors;
    private Double cpr = Double.NaN, rpm = Double.NaN;

    public IndependentMotorGroup(Motor... motors) {
        this.motors = Arrays.asList(motors);
    }

    @NonNull
    @Override
    public Iterator<Motor> iterator() {
        return motors.iterator();
    }

    public void set(double speed) {
        motors.forEach(motor -> motor.set(speed));
    }

    public void stopMotors() {
        motors.forEach(Motor::stopMotor);
    }

    public List<Double> getSpeeds() {
        return motors.stream()
                .map(Motor::get)
                .collect(Collectors.toList());
    }

    public void setDistancePerPulse(double distancePerPulse) {
        motors.forEach(motor -> motor.setDistancePerPulse(distancePerPulse));
    }

    /**
     * @return The position of every motor in units of distance
     */
    public List<Double> getDistances() {
        return motors.stream()
                .map(Motor::getDistance)
                .collect(Collectors.toList());
    }

    /**
     * @return The position of every motor in ticks
     */
    public List<Integer> getPositions() {
        return motors.stream()
                .map(Motor::getCurrentPosition)
                .collect(Collectors.toList());
    }

    public void setRunMode(Motor.RunMode runMode) {
        motors.forEach(motor -> motor.setRunMode(runMode));
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior) {
        motors.forEach(motor -> motor.setZeroPowerBehavior(behavior));
    }

    public void stopAndResetEncoder() {
        motors.forEach(Motor::stopAndResetEncoder);
    }

    public void setGroupType(double cpr, double rpm) {
        this.cpr = cpr;
        this.rpm = rpm;
    }

    public Double getCPR() {
        if (!cpr.isNaN())
            return cpr;

        List<Double> CPRs = motors.stream()
                .map(Motor::getCPR)
                .distinct().collect(Collectors.toList());

        return CPRs.size() == 1 ? CPRs.get(0) : Double.NaN;
    }

    public Double getMaxRPM() {
        if (!rpm.isNaN())
            return rpm;

        List<Double> RPMs = motors.stream()
                .map(Motor::getMaxRPM)
                .distinct().collect(Collectors.toList());

        return RPMs.size() == 1 ? RPMs.get(0) : Double.NaN;
    }

    public void setPositionCoefficient(double kp) {
        motors.forEach(motor -> motor.setPositionCoefficient(kp));
    }

    public void setPositionTolerance(double tolerance) {
        motors.forEach(motor -> motor.setPositionTolerance(tolerance));
    }

    /**
     * @return True if all of the motors reached the target position
     */
    public boolean atTargetPosition() {
        return motors.stream().allMatch(Motor::atTargetPosition);
    }

    /**
     * Sets the target of the motor group in ticks
     *
     * @param target Ticks to travel to
     */
    public void setTargetPosition(int target) {
        motors.forEach(motor -> motor.setTargetPosition(target));
    }

    /**
     * Sets the target of the motor group in the defined distance unit
     *
     * @param target Distance to travel to
     */
    public void setTargetDistance(double target) {
        motors.forEach(motor -> motor.setTargetDistance(target));
    }

    public void setInverted(boolean inverted) {
        motors.forEach(motor -> motor.setInverted(inverted));
    }

    public void disable() {
        motors.forEach(Motor::disable);
    }
}
