package org.firstinspires.ftc.teamcode.experiment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-Centric TeleOp optimized for the Virtual Robot simulator
 * This version adapts our robot code to work with the simulator's hardware mappings
 */
@TeleOp(name="Field Centric TeleOp Simulator", group="Linear Opmode")
public class FieldCentricTeleOpSimulator extends LinearOpMode {

    // Declare OpMode members
    private final ElapsedTime runtime = new ElapsedTime();
    
    // Mecanum drive motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    
    // Arm motors - Using simulator's available motors
    private DcMotor leftArmExtend = null;
    private DcMotor rightArmExtend = null;
    private DcMotor leftArmLift = null;
    private DcMotor rightArmLift = null;
    
    // Servos - Using simulator's available servos
    private Servo leftIntake = null;
    private Servo rightIntake = null;
    
    // IMU for field-centric drive
    private IMU imu = null;
    
    // Constants
    private static final double DRIVE_SPEED_FACTOR = 0.8;
    private static final double ARM_SPEED_FACTOR = 0.5;
    private static final double INTAKE_OPEN_POSITION = 0.8;
    private static final double INTAKE_CLOSED_POSITION = 0.2;
    
    // Simulator specific flags
    private boolean simulateArmMotors = true;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize hardware with simulator mappings
        initializeHardware();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // Reset IMU heading
        imu.resetYaw();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Field-centric drive control
            driveMecanum();
            
            // Arm control
            controlArm();
            
            // Intake control
            controlIntake();
            
            // Show status information
            displayTelemetry();
        }
    }
    
    private void initializeHardware() {
        try {
            // Initialize drive motors with simulator mappings
            frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
            backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
            backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
            
            // Set motor directions
            frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            
            // For simulator: Most configurations only have 4 motors, so we'll:
            // 1. Try to get additional motors if available
            // 2. Otherwise simulate them by logging to telemetry
            try {
                // Try to get extra motors if available in the simulator config
                leftArmExtend = hardwareMap.get(DcMotor.class, "arm_left_extend");
                rightArmExtend = hardwareMap.get(DcMotor.class, "arm_right_extend");
                leftArmLift = hardwareMap.get(DcMotor.class, "arm_left_lift");
                rightArmLift = hardwareMap.get(DcMotor.class, "arm_right_lift");
                
                // Set arm motor directions
                leftArmExtend.setDirection(DcMotorSimple.Direction.FORWARD);
                rightArmExtend.setDirection(DcMotorSimple.Direction.REVERSE);
                leftArmLift.setDirection(DcMotorSimple.Direction.FORWARD);
                rightArmLift.setDirection(DcMotorSimple.Direction.REVERSE);
                
                // Set zero power behavior
                leftArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                // If motors aren't available, we'll simulate them in telemetry
                simulateArmMotors = true;
                telemetry.addData("Note", "Simulating arm motors (not available in hardware map)");
                telemetry.update();
            }
            
            // Initialize servos - Using simulator standard mappings
            try {
                leftIntake = hardwareMap.get(Servo.class, "servo1");
                rightIntake = hardwareMap.get(Servo.class, "servo2");
            } catch (Exception e) {
                telemetry.addData("Warning", "Servos not available in simulator config");
                telemetry.update();
            }
            
            // Initialize IMU - The simulator should have this
            imu = hardwareMap.get(IMU.class, "imu");
            
            // Set motor zero power behavior
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
        } catch (Exception e) {
            telemetry.addData("Hardware Init Error", e.getMessage());
            telemetry.update();
        }
    }
    
    private void driveMecanum() {
        // Get gamepad inputs
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;  // Left/right
        double turn = gamepad1.right_stick_x;   // Rotation
        
        // Apply deadzone
        if (Math.abs(drive) < 0.1) drive = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(turn) < 0.1) turn = 0;
        
        // Get the robot's heading
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        // Rotate the movement direction counter to the robot's rotation
        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
        
        // Apply speed modifiers
        double speed = DRIVE_SPEED_FACTOR;
        if (gamepad1.right_trigger > 0.5) {
            speed *= 0.5; // Slow mode
        } else if (gamepad1.left_trigger > 0.5) {
            speed *= 1.5; // Fast mode (capped at 1.0)
        }
        
        // Calculate motor powers
        double frontLeftPower = (rotY + rotX + turn) * speed;
        double frontRightPower = (rotY - rotX - turn) * speed;
        double backLeftPower = (rotY - rotX + turn) * speed;
        double backRightPower = (rotY + rotX - turn) * speed;
        
        // Normalize motor powers
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                           Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Set motor powers
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        
        // Reset IMU heading when pressing Y button
        if (gamepad1.y) {
            imu.resetYaw();
        }
    }
    
    private void controlArm() {
        // Arm extension control using left stick Y on gamepad2
        double armExtendPower = -gamepad2.left_stick_y * ARM_SPEED_FACTOR;
        
        // Arm lift control using right stick Y on gamepad2
        double armLiftPower = -gamepad2.right_stick_y * ARM_SPEED_FACTOR;
        
        if (simulateArmMotors) {
            // Just report the power values in telemetry if real motors aren't available
            telemetry.addData("Arm Extend Power", "%.2f", armExtendPower);
            telemetry.addData("Arm Lift Power", "%.2f", armLiftPower);
        } else {
            // Control actual motors if they're available
            leftArmExtend.setPower(armExtendPower);
            rightArmExtend.setPower(armExtendPower);
            leftArmLift.setPower(armLiftPower);
            rightArmLift.setPower(armLiftPower);
        }
    }
    
    private void controlIntake() {
        // Skip if servos are not available
        if (leftIntake == null || rightIntake == null) {
            return;
        }
        
        // Intake control using bumpers on gamepad2
        if (gamepad2.right_bumper) {
            // Open intake
            leftIntake.setPosition(INTAKE_OPEN_POSITION);
            rightIntake.setPosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.left_bumper) {
            // Close intake
            leftIntake.setPosition(INTAKE_CLOSED_POSITION);
            rightIntake.setPosition(INTAKE_CLOSED_POSITION);
        }
    }
    
    private void displayTelemetry() {
        // Display drive information
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Drive", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                frontLeftDrive.getPower(), frontRightDrive.getPower(),
                backLeftDrive.getPower(), backRightDrive.getPower());
        
        // Display arm information - either real or simulated
        if (!simulateArmMotors) {
            telemetry.addData("Arm Extend", "L: %.2f, R: %.2f",
                    leftArmExtend.getPower(), rightArmExtend.getPower());
            telemetry.addData("Arm Lift", "L: %.2f, R: %.2f",
                    leftArmLift.getPower(), rightArmLift.getPower());
        }
        
        // Display intake information if servos are available
        if (leftIntake != null && rightIntake != null) {
            telemetry.addData("Intake", "L: %.2f, R: %.2f",
                    leftIntake.getPosition(), rightIntake.getPosition());
        }
        
        // Display robot heading
        telemetry.addData("Robot Heading", "%.1f°",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        
        // Display gamepad inputs for debugging
        telemetry.addData("GP1", "Drive: %.2f, Strafe: %.2f, Turn: %.2f",
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("GP2", "Extend: %.2f, Lift: %.2f, Bumpers: %b/%b",
                -gamepad2.left_stick_y, -gamepad2.right_stick_y, 
                gamepad2.left_bumper, gamepad2.right_bumper);
        
        telemetry.update();
    }
} 