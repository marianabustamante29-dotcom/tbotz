/*
Starter Code for Quad Training Robot 2024
Modified by michaudc 2017, 2023, 2024
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import java.util.List;


@TeleOp(name="MaristBot2025: 14471 With Camera", group="Training")
//@Disabled
public class Teleop_Quad_Training_With_Camera2 extends OpMode {

    // Create instance of MaristBaseRobot2024
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad(); 
    DcMotor shooter;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    
    
    double SHOOTER_VELOCITY = 0;
    double SPEED_CONTROL = 1;
    
    // for Camera
    // Adjust these numbers to suit your robot.
    double DESIRED_DISTANCE = 80.0; //  this is how close the camera should get to the target (inches)
    double DESIRED_HEADING = 80.0; //  this is how close the camera should get to the target (inches)
    double DESIRED_YAW = 0.0; //  this is how close the camera should get to the target (inches)

    
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    
    private double NEW_P = 160;
    private double NEW_I = 3;
    private double NEW_D = 0;
     
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        shooter = robot.leftArm;
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        
       // Set PID for launcher
       PIDCoefficients pidSettings = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
       robot.leftArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
       robot.rightArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        
        // Initialize the Apriltag Detection process
        
        initAprilTag();
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.uptake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
       
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //robot.uptake.setPower(0.5);
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double leftY  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double rightX   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double leftX = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            robot.driveStrafer(leftX, leftY, rightX);
            
            //double distance = desiredTag.ftcPose.range;
            
            //telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        
        else if (gamepad1.right_bumper && targetFound) {
            DESIRED_DISTANCE = 120;
            DESIRED_HEADING = 0; //  this is how close the camera should get to the target (inches)
            DESIRED_YAW = 37; //  this is how close the camera should get to the target (inches)


            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing - DESIRED_HEADING;
            double  yawError        = desiredTag.ftcPose.yaw - DESIRED_YAW;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double leftY  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double rightX   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double leftX = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            robot.driveStrafer(leftX, leftY, rightX);
            
            //double distance = desiredTag.ftcPose.range;
            
            //telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        
            DESIRED_DISTANCE = 80;
            DESIRED_YAW = 0.0; //  this is how close the camera should get to the target (inches)
            DESIRED_HEADING = 0.0; //  this is how close the camera should get to the target (inches)

        }
        
        else {
            double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
            double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
            double rightX = gamepad1.right_stick_x * SPEED_CONTROL;
            double yaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            robot.driveFieldCentric(leftX, leftY, rightX, yaw);
        }
        
    
        if (gamepad1.y) {
            robot.imu.resetYaw();
        }
       
        //shooter
        if (gamepad1.dpad_left) {
            SHOOTER_VELOCITY = -1515;
            robot.leftArm.setVelocity(SHOOTER_VELOCITY);
            robot.rightArm.setVelocity(-SHOOTER_VELOCITY);
        }
        else if (gamepad1.dpad_right) {
            SHOOTER_VELOCITY = -1550;
            robot.leftArm.setVelocity(SHOOTER_VELOCITY);
            robot.rightArm.setVelocity(-SHOOTER_VELOCITY);
        }
        if  (gamepad1.dpad_up) {
            SHOOTER_VELOCITY = 0;
            robot.leftArm.setVelocity(SHOOTER_VELOCITY);
            robot.rightArm.setVelocity(-SHOOTER_VELOCITY);
        }
        robot.leftArm.setVelocity(SHOOTER_VELOCITY);
        robot.rightArm.setVelocity(-SHOOTER_VELOCITY);
        
        
        // intake
        if (gamepad1.a) {
            robot.intake.setPower(-1);
            robot.uptake.setPower(.5);
        }
        else if (gamepad1.x) {
            robot.intake.setPower(1);
            robot.uptake.setPower(-.5);
        }
    
        //no power
        else {
            robot.intake.setPower(0);
            robot.uptake.setPower(0);
        }
        //feeder
        if (gamepad1.b) {
            robot.leftHand.setPosition(1);
        }
        else {
            robot.leftHand.setPosition(0);
        }
        
        double velocityL = robot.leftArm.getVelocity();
        double velocityR = robot.rightArm.getVelocity();
        telemetry.addData("velocityL", velocityL);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        
    }
    
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        
        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}
