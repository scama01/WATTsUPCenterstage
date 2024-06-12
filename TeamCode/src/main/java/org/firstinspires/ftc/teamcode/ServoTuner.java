package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(group = "Utility")
public class ServoTuner extends LinearOpMode {

    public static String SERVO_NAME = "test";
    public static boolean INVERT = false, USE_ANGLE = false;
    public static Double POSITION = 0.0;
    public static Integer MIN_DEGREE = 0, MAX_DEGREE = 300;

    @Override
    public void runOpMode() {
        ServoEx servo = new SimpleServo(hardwareMap, SERVO_NAME,
                MIN_DEGREE, MAX_DEGREE, AngleUnit.DEGREES);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            if (USE_ANGLE && servo.getAngle() != POSITION)
                servo.turnToAngle(POSITION);
            else if (!USE_ANGLE && servo.getPosition() != POSITION)
                servo.setPosition(POSITION);

            if (servo.getInverted() != INVERT)
                servo.setInverted(INVERT);

            telemetry.addData("Servo Name", SERVO_NAME);
            telemetry.addData("Current Angle", servo.getAngle());
            telemetry.addData("Current Position", servo.getPosition());

            telemetry.update();
        }
    }
}
