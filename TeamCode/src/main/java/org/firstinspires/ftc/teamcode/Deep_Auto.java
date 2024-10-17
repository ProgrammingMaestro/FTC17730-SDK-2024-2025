package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "DeepAuto")
public class Deep_Auto extends LinearOpMode {
    private IMU imu_IMU;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;
    static final double TICKS_PER_REV = 366.8;
    static final double MAX_RPM = 458.01;
    static final int COUNTS_PER_MOTOR_REV = 28;
    static final int DRIVE_GEAR_REDUCTION = 13;
    static final double WHEEL_DIAMETER_INCHES = 3.7;
    static double WHEEL_RADIUS = 1.9698; // in
    static double GEAR_RATIO = 13.1; // output (wheel) speed / input (motor) speed
    static double TRACK_WIDTH = 16; // in

    double P_DRIVE_GAIN;
    double P_TURN_GAIN;
    int targetHeading;
    double turnSpeed;
    double driveSpeed;
    double headingError;
    double leftTarget;
    double leftSpeed;
    double rightTarget;
    double rightSpeed;
    int HEADING_THRESHOLD;
    double COUNTS_PER_INCH;

    @Override
    public void runOpMode() {
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontleft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backleft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backright");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

        // Initialize variables
        headingError = 0;
        targetHeading = 0;
        driveSpeed = 0;
        turnSpeed = 0;
        leftSpeed = 0;
        rightSpeed = 0;
        leftTarget = 0;
        rightTarget = 0;

        // COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        // How close must the heading get to the target before moving to next step.
        HEADING_THRESHOLD = 1;

        // Define the Proportional control coefficient (or GAIN) for "heading control".
        P_TURN_GAIN = 0.02;
        P_DRIVE_GAIN = 0.03;

        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // Initialize motor encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData("> Robot Heading", Double.parseDouble(JavaUtil.formatNumber(imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0)));
            telemetry.addLine("> press Start to run program, you can move the robot during initialization to see the heading change.");
            telemetry.addLine("> The heading will reset to 0 when you start the program.");
            telemetry.update();
        }
        waitForStart();

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        imu_IMU.resetYaw();
        if (opModeIsActive()) {
            driveStraight(DRIVE_SPEED, 25, 0);
            turnToHeading(TURN_SPEED, 130);
            holdHeading(TURN_SPEED, 130, 0.5);
            driveStraight(DRIVE_SPEED, 30, 130);

            telemetry.addLine("Path Complete");
            telemetry.update();
            // Pause to display last telemetry message.
            sleep(1000);
        }
    }

    private void driveStraight(double maxDriveSpeed, int distance, int heading) {
        double moveCounts;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = distance * COUNTS_PER_INCH;
            leftTarget = leftFront.getCurrentPosition() + moveCounts;  // Changed leftMaster to leftFront
            rightTarget = rightFront.getCurrentPosition() + moveCounts;  // Changed rightMaster to rightFront

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFront.setTargetPosition((int) leftTarget);
            rightFront.setTargetPosition((int) rightTarget);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // Keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
                // If driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    turnSpeed = turnSpeed * -1;
                }
                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, (int) turnSpeed);
                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void turnToHeading(double maxTurnSpeed, int heading) {
        // Run getSteeringCorrection() once to pre-calculate the current error
        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
        // Keep looping while we are still active, and not on heading.
        while (opModeIsActive() && Math.abs(headingError) > HEADING_THRESHOLD) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Math.min(Math.max(turnSpeed, -maxTurnSpeed), maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobot(0, (int) turnSpeed);
            // Display drive status for the driver, false means we are not driving straight.
            sendTelemetry(false);
        }
    }

    private void holdHeading(double maxTurnSpeed, int heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Keep looping while we have time remaining.
        while (opModeIsActive() && holdTimer.seconds() < holdTime) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Math.min(Math.max(turnSpeed, -maxTurnSpeed), maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobot(0, (int) turnSpeed);
            // Display drive status for the driver, false means we are not driving straight.
            sendTelemetry(false);
        }
        // Stop all motion
        moveRobot(0, 0);
    }

    private double getSteeringCorrection(int targetHeading, double gain) {
        double robotHeading = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // Updated for new API
        headingError = targetHeading - robotHeading;
        // Normalize the error to be between -180 and +180
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        // Return the calculated correction factor
        return headingError * gain;
    }

    private void moveRobot(double drive, int turn) {
        leftSpeed = drive + turn;
        rightSpeed = drive - turn;

        leftFront.setPower(leftSpeed);
        leftBack.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);
        rightFront.setPower(rightSpeed);
    }

    private void sendTelemetry(boolean isDrivingStraight) {
        telemetry.addData("Left Front Target", leftFront.getTargetPosition());
        telemetry.addData("Left Back Target", leftBack.getTargetPosition());
        telemetry.addData("Right Front Target", rightFront.getTargetPosition());
        telemetry.addData("Right Back Target", rightBack.getTargetPosition());
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Current Heading", imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // Updated for new API
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.update();
    }
}
