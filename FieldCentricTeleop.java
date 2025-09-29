/**
 * Field-Centric TeleOp with Road Runner + IMU Reset
 * Robot: intake - belt - tilt - outtake - kick system
 * Author: Anthony Kongoasa
 * need to  tune RR before I can test
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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="Field Centric TeleOp (School IMU)", group="Drive")
public class FieldCentricTeleOp extends LinearOpMode {

    // Hardware
    private IMU imu;
    private DcMotor outtake, tilt, belt, intake;
    private Servo kick; // TODO rename to gate?

    // Arm control fields
    private int armPos = 0;
    private double ARM_SPEED = 0.0;

    // Deadzone helper
    private double applyDeadzone(double value, double dz) {
        return Math.abs(value) < dz ? 0.0 : value;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Hardware init
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        tilt = hardwareMap.get(DcMotor.class, "tilt");
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kick = hardwareMap.get(Servo.class, "kick");
        kick.setPosition(0.8); //close

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
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            belt.setPower(0.5);

            // Get current heading
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double heading = orientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

            // Joystick inputs
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // turn

            // Deadzone
            y = applyDeadzone(y, 0.05);
            x = applyDeadzone(x, 0.05);
            rx = applyDeadzone(rx, 0.05);

            // Field-centric transform
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;

            // Drive
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rx));
            drive.update();

            // Reset yaw
            if (gamepad1.y) {
                imu.resetYaw();
            }

            // Intake 
            double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
            intake.setPower(intakePower);

            // Arm tilt
            double armPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (armPower > 0.1) {
                ARM_SPEED = 0.7;
                armPos += 10; // adjust step size
            } else if (armPower < -0.1) {
                ARM_SPEED = 0.7;
                armPos -= 10;
            }

            // Clamp armPos to prevent over-extension
            armPos = Math.max(0, Math.min(armPos, 1000)); // replace 1000 with your max encoder ticks

            tilt.setTargetPosition(armPos);
            tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tilt.setPower(ARM_SPEED);

            // Shoot
           
            if (gamepad1.x) {
                outtake.setPower(1);
            }
            else {
                outtake.setPower(0);
            }
            
            if (gamepad1.dpad_up) {
                
                kick.setPosition(0.8);  //open
                belt.setPower(1); //push
            }
                
            } else if (gamepad1.dpad_down) {
               
                kick.setPosition(0.3); // close
                belt.setPower(0.6);
            }

            // Telemetry
            telemetry.addData("Heading (rad)", heading);
            telemetry.addData("Arm target ticks", armPos);
            telemetry.update();
        }
    }
}
