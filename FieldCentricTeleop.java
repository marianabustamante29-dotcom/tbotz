/**
 * Field-Centric TeleOp with Road Runner + IMU + AprilTag Alignment
 * Robot: intake - belt - dual outtakes - kick system
 * Author: Anthony Kongoasa
 */

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Field Centric TeleOp (IMU + AprilTag)", group="Drive")
public class FieldCentricTeleOp extends LinearOpMode {

    // Hardware
    private IMU imu;
    private DcMotor belt, intake;
    private DcMotorEx leftShooter, rightShooter;
    private Servo kick;

    // Shooter control
    private boolean shooterRunning = false;
    private boolean xLast = false;
    private boolean reachedTarget = false;
    //private double targetVel = 2000; // Target RPM
    private final double SHOOTER_TOLERANCE = 50;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Deadzone helper
    private double applyDeadzone(double value, double dz) {
        return Math.abs(value) < dz ? 0.0 : value;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Hardware init
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kick = hardwareMap.get(Servo.class, "kick");
        kick.setPosition(0.8); // up/closed

        // IMU init
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Motor modes
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Vision init
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            // Get IMU heading
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double heading = orientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

            // Joystick inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Deadzone
            y = applyDeadzone(y, 0.05);
            x = applyDeadzone(x, 0.05);
            rx = applyDeadzone(rx, 0.05);

            // Field-centric transform
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;

            // Auto-align to AprilTag if left bumper held
            if (gamepad1.left_bumper) {
                AprilTagDetection targetTag = null;
                List<AprilTagDetection> detections = aprilTag.getDetections();
                for (AprilTagDetection det : detections) {
                    if (det.metadata != null) {
                        targetTag = det;
                        break;
                    }
                }
                if (targetTag != null) {
                    // adjust rotation based on tag bearing
                    rx = targetTag.ftcPose.bearing * 0.01; // tune gain
                    rx = Math.max(-0.5, Math.min(0.5, rx)); // smooth clipping
                }
            }

            // Send drive powers to Road Runner
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rx));
            drive.updatePoseEstimate();

            // Intake control
            if (gamepad1.a) intake.setPower(0.8);
            else intake.setPower(0);

            // Shooter toggle
            boolean xNow = gamepad1.x;
            if (xNow && !xLast) shooterRunning = !shooterRunning;
            xLast = xNow;

            if (shooterRunning) {
                leftShooter.setVelocity(targetVel);
                rightShooter.setVelocity(-targetVel); // reversed
                // check if velocity reached
                reachedTarget = Math.abs(leftShooter.getVelocity() - targetVel) < SHOOTER_TOLERANCE;
            } else {
                leftShooter.setVelocity(0);
                rightShooter.setVelocity(0);
                reachedTarget = false;
            }

            // Kick control
            if (gamepad1.dpad_up && reachedTarget) {
                kick.setPosition(0.1); // up
                belt.setPower(1.0);    // push
            } else if (gamepad1.dpad_down) {
                kick.setPosition(0.3); // down/kick
                belt.setPower(0.6);
            } else {
                belt.setPower(0);
            }

            // Reset yaw
            if (gamepad1.y) imu.resetYaw();

            // Telemetry
            telemetry.addData("Heading (rad)", heading);
            telemetry.addData("LeftShooterVel", leftShooter.getVelocity());
            telemetry.addData("RightShooterVel", rightShooter.getVelocity());
            telemetry.addData("TargetVel", targetVel);
            telemetry.addData("ReachedTarget", reachedTarget);
            telemetry.update();
        }
    }
}
