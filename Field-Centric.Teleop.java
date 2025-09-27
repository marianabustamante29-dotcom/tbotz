/**
 * Field Centric TeleOp with Road Runner + IMU Reset
 * Example robot: intake - belt - outtake system
 * Author: Anthony Kongoasa
 */

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="Field Centric TeleOp")
public class FieldCentricTeleOpRR_ResetYaw extends LinearOpMode {

    private IMU imu;
    private DcMotor outtake;
    private DcMotor tilt;
    private DcMotor belt;
    private DcMotor intake;
    private Servo kick;

    private int armPos = 0;          // encoder target for tilt motor
    private double ARM_SPEED = 0.0;  // motor speed for tilt

    private double applyDeadzone(double value, double dz) {
        return Math.abs(value) < dz ? 0.0 : value;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Hardware init
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");
        tilt = hardwareMap.get(DcMotor.class, "tilt");
        kick = hardwareMap.get(Servo.class, "kick");

        // IMU setup with REV Hub orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Motor modes
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // Current heading (yaw)
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double heading = orientation.getYaw(AngleUnit.RADIANS);

            // Joysticks
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotate

            // Deadzones
            y = applyDeadzone(y, 0.05);
            x = applyDeadzone(x, 0.05);
            rx = applyDeadzone(rx, 0.05);

            // Field-centric transform
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;

            // Road Runner drive power
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rx));
            drive.update();

            // Reset yaw with Y
            if (gamepad1.y) {
                imu.resetYaw();
            }

            // Intake
            double inPower = gamepad2.right_trigger - gamepad2.left_trigger;
            intake.setPower(inPower);

            // Arm tilt (RUN_TO_POSITION)
            double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (armMotorPower > 0.1) {
                ARM_SPEED = 0.7;
                armPos += 10; // adjust step size
            } else if (armMotorPower < -0.1) {
                ARM_SPEED = 0.7;
                armPos -= 10;
            }
            tilt.setTargetPosition(armPos);
            tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tilt.setPower(ARM_SPEED);

            // Outtake
            if (gamepad1.a) {
                outtake.setPower(1.0);
            } else {
                outtake.setPower(0);
            }

            // Kicker servo
            if (gamepad1.dpad_up) {
                kick.setPosition(0.8);
            } else if (gamepad1.dpad_down) {
                kick.setPosition(0.3);
            }

            // Telemetry
            telemetry.addData("Heading (rad)", heading);
            telemetry.addData("Arm target", armPos);
            telemetry.update();
        }
    }
}
