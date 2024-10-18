/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcMatchInfo;
import ftclib.robotcore.FtcOpMode;

import java.util.Arrays;

import ftclib.motor.FtcDcMotor;
import ftclib.sensor.FtcOctoQuad;
import ftclib.sensor.FtcRobotBattery;
import ftclib.sensor.FtcSparkFunOtos;
import autotasks.TaskAutoPlacePixel;
import drivebases.MecanumDrive;
import drivebases.RobotDrive;
import drivebases.SwerveDrive;
import subsystems.BlinkinLEDs;
import subsystems.ElevatorArm;
import subsystems.Intake;
import subsystems.PixelTray;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcDigitalInput;
import trclib.timer.TrcTimer;
import vision.Vision;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    private static final String moduleName = Robot.class.getSimpleName();
    private static final double STATUS_UPDATE_INTERVAL = 0.1;   // 100 msec
    //
    // Global objects.
    //
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    private static double nextStatusUpdateTime = 0.0;
    //
    // Vision subsystems.
    //
    public Vision vision;
    //
    // Sensors and indicators.
    //
    public BlinkinLEDs blinkin;
    public FtcRobotBattery battery;
    public FtcOctoQuad octoQuad;
    public FtcSparkFunOtos sparkfunOtos;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive;
    public ElevatorArm elevatorArm;
    public Intake intake;
    public PixelTray pixelTray;
    public FtcDcMotor launcher;

    public TaskAutoPlacePixel placePixelTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        nextStatusUpdateTime = TrcTimer.getCurrentTime();

        speak("Init starting");
        //
        // Initialize vision subsystems.
        //
        if (RobotParams.Preferences.tuneColorBlobVision ||
            RobotParams.Preferences.useAprilTagVision ||
            RobotParams.Preferences.useColorBlobVision ||
            RobotParams.Preferences.useTensorFlowVision)
        {
            vision = new Vision(this);
        }

        if (RobotParams.Preferences.useOctoQuad)
        {
            octoQuad = new FtcOctoQuad(RobotParams.HWNAME_OCTOQUAD, 0);
        }

        if (RobotParams.Preferences.useSparkfunOtos)
        {
            sparkfunOtos = new FtcSparkFunOtos(RobotParams.HWNAME_SPARKFUNOTOS);
        }
        //
        // If robotType is NoRobot, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (RobotParams.Preferences.robotType != RobotParams.RobotType.NoRobot)
        {
            //
            // Create and initialize sensors and indicators.
            //
            if (RobotParams.Preferences.useBlinkin)
            {
                blinkin = new BlinkinLEDs(RobotParams.HWNAME_BLINKIN);
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize RobotDrive.
            //
            robotDrive =
                RobotParams.Preferences.robotType == RobotParams.RobotType.SwerveRobot?
                    new SwerveDrive(): new MecanumDrive();
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useElevatorArm)
                {
                    elevatorArm = new ElevatorArm();
                    // Code review: Should this init be in Robot.startMode?
                    if (runMode == TrcRobot.RunMode.TELEOP_MODE)
                    {
                        elevatorArm.wristSetPosition(RobotParams.WRIST_DOWN_POS);
                    }
                }

                if (RobotParams.Preferences.useIntake)
                {
                    intake = new Intake(RobotParams.HWNAME_INTAKE, this);
                }

                if (RobotParams.Preferences.usePixelTray)
                {
                    pixelTray = new PixelTray(RobotParams.HWNAME_PIXELTRAY);
                    // Code review: Should this init be in Robot.startMode?
                    if (runMode == TrcRobot.RunMode.TELEOP_MODE)
                    {
                        pixelTray.setUpperGateOpened(true, null);
                        pixelTray.setLowerGateOpened(true, null);
                    }
                    else
                    {
                        pixelTray.setUpperGateOpened(false, null);
                        pixelTray.setLowerGateOpened(false, null);
                    }
                }

                if (RobotParams.Preferences.useLauncher)
                {
                    launcher = new FtcDcMotor(RobotParams.HWNAME_LAUNCHER);
                    launcher.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
                    launcher.setMotorInverted(RobotParams.LAUNCHER_MOTOR_INVERTED);
                }
                //
                // Create auto tasks here.
                //
                placePixelTask = new TaskAutoPlacePixel("PlacePixelTask", this);
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return RobotParams.ROBOT_NAME;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE)
            {
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);

        Arrays.fill(totalElapsedTime, 0L);
        loopCount = 0;
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        if (loopCount > 0)
        {
            globalTracer.traceInfo(
                moduleName,
                "Update Status average elapsed times:\n" +
                "DriveBase=%.6fs\n" +
                " Elevator=%.6fs\n" +
                "      Arm=%.6fs\n" +
                "    Wrist=%.6fs\n" +
                "   Intake=%.6fs\n" +
                "PixelTray=%.6fs\n",
                totalElapsedTime[0]/1000000000.0/loopCount,     //DriveBase
                totalElapsedTime[1]/1000000000.0/loopCount,     //Elevator
                totalElapsedTime[2]/1000000000.0/loopCount,     //Arm
                totalElapsedTime[3]/1000000000.0/loopCount,     //Wrist
                totalElapsedTime[4]/1000000000.0/loopCount,     //Intake
                totalElapsedTime[4]/1000000000.0/loopCount);    //PixelTray
        }
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.purplePixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling PurplePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.PurplePixel, false);
            }

            if (vision.greenPixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling GreenPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.GreenPixel, false);
            }

            if (vision.yellowPixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling YellowPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.YellowPixel, false);
            }

            if (vision.whitePixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling WhitePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.WhitePixel, false);
            }

            if (vision.redBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RedBlobVision.");
                vision.setRedBlobVisionEnabled(false);
            }

            if (vision.blueBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling BlueBlobVision.");
                vision.setBlueBlobVisionEnabled(false);
            }

            if (vision.tensorFlowVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling TensorFlowVision.");
                vision.setTensorFlowVisionEnabled(false);
            }

            vision.close();
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    private final long[] totalElapsedTime = new long[6];
    private long loopCount;

    /**
     * This method update all subsystem status on the dashboard.
     */
    public void updateStatus()
    {
        if (TrcTimer.getCurrentTime() > nextStatusUpdateTime)
        {
            int lineNum = 1;
            long startNanoTime;

            nextStatusUpdateTime += STATUS_UPDATE_INTERVAL;
            if (robotDrive != null)
            {
                startNanoTime = TrcTimer.getNanoTime();
                dashboard.displayPrintf(
                    ++lineNum,
                    "DriveBase: Pose=" + robotDrive.driveBase.getFieldPosition());
                totalElapsedTime[0] += TrcTimer.getNanoTime() - startNanoTime;
            }

            if (elevatorArm != null)
            {
                if (elevatorArm.elevator != null)
                {
                    startNanoTime = TrcTimer.getNanoTime();
                    dashboard.displayPrintf(
                        ++lineNum,
                        "Elevator: power=" + elevatorArm.elevator.getPower() +
                        ",pos=" + elevatorArm.elevator.getPosition() +
                        ",target=" + elevatorArm.elevator.getPidTarget() +
                        ",lowerLimitSw=" + elevatorArm.elevator.isLowerLimitSwitchActive());
                    totalElapsedTime[1] += TrcTimer.getNanoTime() - startNanoTime;
                }

                if (elevatorArm.arm != null)
                {
                    startNanoTime = TrcTimer.getNanoTime();
                    dashboard.displayPrintf(
                        ++lineNum,
                        "Arm: power=" + elevatorArm.arm.getPower() +
                        ",pos=" + elevatorArm.arm.getPosition() + "/" + elevatorArm.arm.getEncoderRawPosition() +
                        ",target=" + elevatorArm.arm.getPidTarget());
                    totalElapsedTime[2] += TrcTimer.getNanoTime() - startNanoTime;
                }

                if (elevatorArm.wrist != null)
                {
                    startNanoTime = TrcTimer.getNanoTime();
                    if (elevatorArm.wristSensor != null)
                    {
                        dashboard.displayPrintf(
                            ++lineNum,
                            "Wrist: pos=" + elevatorArm.wrist.getPosition() +
                            ",distance=" + elevatorArm.wristGetDistance());
                    }
                    else
                    {
                        dashboard.displayPrintf(++lineNum, "Wrist: pos=" + elevatorArm.wrist.getPosition());
                    }
                    totalElapsedTime[3] += TrcTimer.getNanoTime() - startNanoTime;
                }
            }

            if (intake != null)
            {
                startNanoTime = TrcTimer.getNanoTime();
                dashboard.displayPrintf(
                    ++lineNum,
                    "Intake: power=" + intake.getIntakeMotor().getPower() +
                    ",sensor=" + intake.getDistance() +
                    ",has2Pixels=" + intake.hasTwoPixels());
                totalElapsedTime[4] += TrcTimer.getNanoTime() - startNanoTime;
            }

            if (pixelTray != null)
            {
                startNanoTime = TrcTimer.getNanoTime();
                dashboard.displayPrintf(
                    ++lineNum,
                    "PixelTray: lowerGateOpened=" + pixelTray.isLowerGateOpened() +
                    ",upperGateOpened=" + pixelTray.isUpperGateOpened());
                totalElapsedTime[5] += TrcTimer.getNanoTime() - startNanoTime;
            }
            loopCount++;
        }
    }   //updateStatus

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (elevatorArm != null)
        {
            elevatorArm.zeroCalibrate(owner);
        }
    }   //zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
        robotDrive.driveBase.setFieldPosition(
            adjustPoseByAlliance(
                autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                    RobotParams.STARTPOS_BLUE_AUDIENCE: RobotParams.STARTPOS_BLUE_BACKSTAGE,
                autoChoices.alliance, false));
    }   //setRobotStartPosition

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            double angleDelta = (newPose.angle - 90.0) * 2.0;
            newPose.angle -= angleDelta;
            newPose.y = -newPose.y;
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.FULL_TILE_INCHES;
            newPose.y *= RobotParams.FULL_TILE_INCHES;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, true);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose, alliance, true);
    }   //adjustPoseByAlliance

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
