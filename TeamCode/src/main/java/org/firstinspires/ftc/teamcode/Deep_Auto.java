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

@Autonomous(name = "DeepAuto (Blocks to Java)")
public class Deep_Auto extends LinearOpMode {

    private IMU imu_IMU;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;

    int targetHeading;
    double turnSpeed;
    double driveSpeed;
    double headingError;
    double leftTarget;
    double P_DRIVE_GAIN;
    double leftSpeed;
    double COUNTS_PER_INCH;
    double rightTarget;
    double rightSpeed;
    int HEADING_THRESHOLD;
    double P_TURN_GAIN;
    @Override
    public void runOpMode() {
        int COUNTS_PER_MOTOR_REV;
        int DRIVE_GEAR_REDUCTION;
        double WHEEL_DIAMETER_INCHES;
        double DRIVE_SPEED;
        double TURN_SPEED;

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

        COUNTS_PER_MOTOR_REV = 28;

        DRIVE_GEAR_REDUCTION = 13;

        WHEEL_DIAMETER_INCHES = 3.7;
        // COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        // Max driving speed for better distance accuracy.
        DRIVE_SPEED = 0.6;
        // Max Turn speed to limit turn rate.
        TURN_SPEED = 0.4;
        // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
        HEADING_THRESHOLD = 1;
        // Define the Proportional control coefficient (or GAIN) for "heading control".
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
        // Larger is more responsive, but also less stable
        P_TURN_GAIN = 0.02;
        // Larger is more responsive, but also less stable
        P_DRIVE_GAIN = 0.03;

        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse the motors
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
            waitForStart();
        }
    }
    /*
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
            leftTarget = leftMaster.getCurrentPosition() + moveCounts;
            rightTarget = rightMaster.getCurrentPosition() + moveCounts;
            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftMaster.setTargetPosition((int) leftTarget);
            rightMaster.setTargetPosition((int) rightTarget);
            leftMaster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMaster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && leftMaster.isBusy() && rightMaster.isBusy()) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    turnSpeed = turnSpeed * -1;
                }
                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, (int) turnSpeed);
                // Display drive status for the driver.
                // Call with True to indicate we're driving straight.
                sendTelemetry(true);
            }
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    private void turnToHeading(double maxTurnSpeed, int heading) {
        // Run getSteeringCorrection() once to pre-calculate the current error
        turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
        // keep looping while we are still active, and not on heading.
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
        ElapsedTime holdTimer;

        holdTimer = new ElapsedTime();
        holdTimer.reset();
        // keep looping while we have time remaining.
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
        //  Stop all motion
        moveRobot(0, 0);
    }


    private void moveRobot(double drive, int turn) {
        double max;

        // save drive/turn values so they can be used by telemetry
        driveSpeed = drive;
        turnSpeed = turn;
        leftSpeed = drive - turn;
        rightSpeed = drive + turn;
        // Scale speeds down if either one exceeds +/- 1.0
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftSpeed), Math.abs(rightSpeed)));
        if (max > 1) {
            leftSpeed = leftSpeed / max;
            rightSpeed = rightSpeed / max;
        }
        leftMaster.setPower(leftSpeed);
        rightMaster.setPower(rightSpeed);
    }


    private double getSteeringCorrection(int desiredHeading, double proportionalGain) {
        // Save for telemetry
        targetHeading = desiredHeading;
        // Determine the heading current error
        headingError = targetHeading - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) {
            headingError = headingError - 360;
        }
        while (headingError <= -180) {
            headingError = headingError + 360;
        }
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0 and return the result.
        return Math.min(Math.max(headingError * proportionalGain, -1), 1);
    }


    private double getHeading() {
        // read the Robot heading directly from the IMU (in degrees) and return that value
        return imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", Double.parseDouble(JavaUtil.formatNumber(leftTarget, 0)) + " : " + Double.parseDouble(JavaUtil.formatNumber(rightTarget, 0)));
            telemetry.addData("Actual Pos L:R", Double.parseDouble(JavaUtil.formatNumber(leftMaster.getCurrentPosition(), 0)) + " : " + Double.parseDouble(JavaUtil.formatNumber(rightMaster.getCurrentPosition(), 0)));
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("Heading- Target : Current", Double.parseDouble(JavaUtil.formatNumber(targetHeading, 2)) + " : " + Double.parseDouble(JavaUtil.formatNumber(getHeading(), 2)));
        telemetry.addData("Error  : Steer Pwr", Double.parseDouble(JavaUtil.formatNumber(headingError, 1)) + ":" + Double.parseDouble(JavaUtil.formatNumber(turnSpeed, 1)));
        telemetry.addData("Wheel Speeds L : R", Double.parseDouble(JavaUtil.formatNumber(leftSpeed, 2)) + " : " + Double.parseDouble(JavaUtil.formatNumber(rightSpeed, 2)));
        telemetry.update();
    }
}
*/
}
