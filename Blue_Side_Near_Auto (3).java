package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;


@Autonomous(name="Blue_side_near_auto", group="Test Autos")
//@Disabled
public class Blue_Side_Near_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad();   
    
    private ElapsedTime runtime = new ElapsedTime();

    // Declare fields for Odometry Pod
    GoBildaPinpointDriver odo;
    double oldTime = 0;

    // Gains for PID
    final double DESIRED_DISTANCE = 10.0;
    final double SPEED_GAIN = 0.015;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN = 0.05;

    // Max Speeds
    final double MAX_AUTO_SPEED = 0.6;
    final double MAX_AUTO_STRAFE = 0.6;
    final double MAX_AUTO_TURN = 0.6;
        

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(160, -90, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetRuntime();
        
        // Navigate to Points on the Field
        Pose2D targetPos1 = new Pose2D(DistanceUnit.INCH, 35, 36, AngleUnit.DEGREES, 45);
        Pose2D targetPos2 = new Pose2D(DistanceUnit.INCH, 38, 25, AngleUnit.DEGREES, -90);
        Pose2D targetPos3 = new Pose2D(DistanceUnit.INCH, 38, -4, AngleUnit.DEGREES, -90);
        Pose2D targetPos4 = new Pose2D(DistanceUnit.INCH, 62, 25, AngleUnit.DEGREES, -90);
        Pose2D targetPos5 = new Pose2D(DistanceUnit.INCH, 62, -10.5, AngleUnit.DEGREES, -90);
        Pose2D targetPos9 = new Pose2D(DistanceUnit.INCH, 35, 15, AngleUnit.DEGREES, -90);
        // intake 3 and go to the lever
        Pose2D targetPos6 = new Pose2D(DistanceUnit.INCH, 87, 25, AngleUnit.DEGREES, -90);
        Pose2D targetPos7 = new Pose2D(DistanceUnit.INCH, 87, -10, AngleUnit.DEGREES, -90);
        Pose2D targetPos8 = new Pose2D(DistanceUnit.INCH, 55,  15, AngleUnit.DEGREES,  -90);

        
        
        
        //actual code
        
        robot.leftArm.setVelocity(-1130);
        robot.rightArm.setVelocity(1130);
        goToPose(targetPos1, 5, 3);
        delay(0.1);
        //SHOOT
        shoot();
        goToPose(targetPos2, 5, 3);
        delay(0.1);
        //intake
        goToPose(targetPos3, 5, 3);
        robot.intake.setPower(0);
        //shooting position
        goToPose(targetPos1, 5, 3);
        delay(0.1);
        //shoot
        shoot();
        goToPose(targetPos4, 5, 3);
        delay(0.1);
        //intake
        goToPose(targetPos5, 5, 3);
        robot.intake.setPower(0);
        delay(0.1);
        goToPose(targetPos4, 5, 3);
        goToPose(targetPos1, 5, 3);
        delay(0.1);
        //shoot
        shoot();
        goToPose(targetPos9, 5, 3);
        
        // Intake 3 more and line up by the lever
        /* 
        goToPose(targetPos6, 5, 3);
        delay(0.1);
        goToPose(targetPos7, 5, 3);
        delay(0.1);
        goToPose(targetPos6, 5, 3);
        goToPose(targetPos8, 5, 3);
        */

        // Autonomous Finished
        telemetry.addData("Path", "C omplete");
        telemetry.update();
        //sleep(1000);
    }
    
    public void shoot() {
        
        robot.intake.setPower(-1);
        robot.rightUptake.setPosition(1);
        robot.leftUptake.setPosition(0);
        delay(.000000000001);
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(1);
        delay(.25);
        robot.leftHand.setPosition(.5);
        robot.rightHand.setPosition(.5);
        delay(.5);
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(1);
        delay(.25);
        robot.leftHand.setPosition(.5);
        robot.rightHand.setPosition(.5);
        delay(1);
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(1);
        delay(.25);
        robot.leftHand.setPosition(.5);
        robot.rightHand.setPosition(.5);
        delay(.95);
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(1);
        delay(.25);
        robot.leftHand.setPosition(.5);
        robot.rightHand.setPosition(.5);
        delay(0.00000000001);
    }
    
    // Sample Delay Code
    // t is in seconds
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }

    // Method to go to a Pose2D Point
    public void goToPose(Pose2D targetPos, double threshold, double timeout) {

        // Current Data
        Pose2D currentPos = odo.getPosition();
        double xPos = currentPos.getX(DistanceUnit.MM);
        double yPos = currentPos.getY(DistanceUnit.MM);
        double thetaPos = currentPos.getHeading(AngleUnit.DEGREES);

        // Target Data
        double xTarget = targetPos.getX(DistanceUnit.MM);
        double yTarget = targetPos.getY(DistanceUnit.MM);
        double thetaTarget = targetPos.getHeading(AngleUnit.DEGREES);

        double drive = 0;
        double turn = 0;
        double strafe = 0;

        // TODO: Figure out Exit
        // For Now Using Threshold of distance
        
        runtime.reset();
        while(opModeIsActive() && getPoseDistance(currentPos, targetPos) > threshold && (runtime.seconds() < timeout) ) {
            odo.update();
            
            currentPos = odo.getPosition();
            xPos = currentPos.getX(DistanceUnit.MM);
            yPos = currentPos.getY(DistanceUnit.MM);
            thetaPos = currentPos.getHeading(AngleUnit.DEGREES);

            // Compute Differences
            double xError = xPos - xTarget;
            double yError = yPos - yTarget;
            double yawError = thetaPos - thetaTarget;

            drive = Range.clip(xError*SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(yawError*TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(yError*STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            robot.driveFieldCentric(strafe, drive, turn, thetaPos);

        }

        robot.driveFieldCentric(0, 0, 0, 0);

    }

    private double getPoseDistance(Pose2D pos1, Pose2D pos2) {
        double x1 = pos1.getX(DistanceUnit.MM);
        double y1 = pos1.getY(DistanceUnit.MM);
        double theta1 = pos1.getHeading(AngleUnit.DEGREES);

        double x2 = pos2.getX(DistanceUnit.MM);
        double y2 = pos2.getY(DistanceUnit.MM);
        double theta2 = pos2.getHeading(AngleUnit.DEGREES);

        double diff = Math.pow((x1-x2), 2) + Math.pow((y1-y2), 2) + Math.pow((theta1 - theta2), 2);
        double distance = Math.sqrt(diff);

        return distance;

    }
    
    
}


