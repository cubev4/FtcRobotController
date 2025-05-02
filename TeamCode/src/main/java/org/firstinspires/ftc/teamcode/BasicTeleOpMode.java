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


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            double armMotorPower=0;
            double armExtenderPower=0;

            if (currentGamepad1.a && !previousGamepad1.a) {
                intakeToggle = !intakeToggle;
            }

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
                intakeToggle = !intakeToggle;
            }

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

            if (gamepad1.left_bumper) {
                armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotorPower = 1;
            } else {
                if(gamepad1.right_bumper) {
                    armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
                    armMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                    armMotorPower = 1;
                } else {
                    armMotorPower = 0;
                }
            }
            if (gamepad1.left_trigger>0 ) {
                armExtender1.setDirection(DcMotorSimple.Direction.REVERSE);
                armExtender2.setDirection(DcMotorSimple.Direction.REVERSE);
                armExtenderPower= 1;
            } else {
                if(gamepad1.right_trigger>0) {
                    armExtender1.setDirection(DcMotorSimple.Direction.FORWARD);
                    armExtender2.setDirection(DcMotorSimple.Direction.FORWARD);
                    armExtenderPower = 1;
                } else {
                    armExtenderPower = 0;
                }
            }
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