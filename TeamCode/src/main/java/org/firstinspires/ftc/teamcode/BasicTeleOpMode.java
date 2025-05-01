package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class BasicTeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armMotor1 = hardwareMap.dcMotor.get("armMotor1");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("armMotor2");
        DcMotor armExtender1 =hardwareMap.dcMotor.get("armExtender1");
        DcMotor armExtender2 =hardwareMap.dcMotor.get("armExtender2");
        Servo activeIntake1 = hardwareMap.get(Servo.class,"activeIntake1");
        Servo activeIntake2 = hardwareMap.get(Servo.class,"activeIntake2");

        boolean intakeToggle = false;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // other initialization code goes here

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            double armMotorPower=1;
            double armExtenderPower=1;
            // Rising edge detect

            if (currentGamepad1.a && !previousGamepad1.a) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                intakeToggle = !intakeToggle;
            }

            // Using the toggle variable to control the robot.
            if (intakeToggle) {
                activeIntake1.setDirection(Servo.Direction.FORWARD);
                activeIntake2.setDirection(Servo.Direction.FORWARD);
                activeIntake1.setPosition(1);
                activeIntake2.setPosition(1);
            }
            else {
                activeIntake1.setPosition(0);
                activeIntake2.setPosition(0);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                intakeToggle = !intakeToggle;
            }

// Using the toggle variable to control the robot.
            if (intakeToggle) {
                activeIntake1.setDirection(Servo.Direction.REVERSE);
                activeIntake2.setDirection(Servo.Direction.REVERSE);
                activeIntake1.setPosition(1);
                activeIntake2.setPosition(1);
            }
            else {
                activeIntake1.setPosition(0);
                activeIntake2.setPosition(0);
            }

// Using the toggle variable to control the robot.
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            frontRightMotor.setPower(frontRightPower);

            armMotor1.setPower(armMotorPower);
            armMotor2.setPower(armMotorPower);
            armExtender1.setPower(armExtenderPower);
            armExtender2.setPower(armExtenderPower);
        }
    }
}