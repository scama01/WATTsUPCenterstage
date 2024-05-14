package org.firstinspires.ftc.teamcode.util;

import android.util.Pair;

import com.arcrobotics.ftclib.hardware.ServoEx;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.stream.Stream;

/**
 * <p>Servomotor wrapper that uses cubic spline interpolation to restrict itself to a relevant range.</p>
 * <p>Can be used to synchronize servomotors together or to mimic real angles to account for inaccurate potentiometer measurements/angle conversions.</p>
 **/
public class InterpolatedServo {
    private final ServoEx servo;
    private double currentPosition = 0.0;
    private PolynomialSplineFunction interpolation;

    public InterpolatedServo(ServoEx servo) {
        this.servo = servo;
    }

    /**
     * <p>Interpolates the servo's position to a relevant angle range using a cubic spline.</p>
     * <p>Typical usage consists of restricting the servomotor to certain angles with a range from 0 to 1.</p>
     *
     * @param controlPoints A list of input : output pairs for the generator to use (must use at least 2 pairs)
     */
    @SafeVarargs
    public final void generatePositions(Pair<Double, Double>... controlPoints) {
        double[] x = Stream.of(controlPoints)
                .mapToDouble(p -> p.first)
                .toArray();
        double[] y = Stream.of(controlPoints)
                .mapToDouble(p -> p.second)
                .toArray();

        if (x.length == 2)
            interpolation = new LinearInterpolator().interpolate(x, y);
        else interpolation = new SplineInterpolator().interpolate(x, y);
    }

    /**
     * Sets the position of the servomotor based on its interpolated function.
     *
     * @param position The position to set the servo to
     * @throws RuntimeException         If the positions weren't generated
     * @throws IllegalArgumentException If the desired position is out of the generated range
     */
    public void setToPosition(double position) throws RuntimeException {
        if (interpolation == null)
            throw new RuntimeException("The positions of the servomotor weren't generated.");

        if (!interpolation.isValidPoint(position)) {
            double[] knots = interpolation.getKnots();
            throw new IllegalArgumentException(String.format("Unable to access position %.2f. " +
                    "Spline positions range from [%.2f, %.2f]", position, knots[0], knots[knots.length - 1]));
        }
        servo.turnToAngle(interpolation.value(position));
        currentPosition = position;
    }

    public void setInverted(boolean isInverted) {
        servo.setInverted(isInverted);
    }

    public Double getCurrentPosition() {
        return currentPosition;
    }
}
