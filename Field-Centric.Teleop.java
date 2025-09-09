//for roadrunner/android studio
// assuming we use the intake - belt - outtake robot system

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.hardware.sensor.IMU; 
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;  
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Field Centric TeleOp")
public class FieldCentricTeleOpRR_ResetYaw extends LinearOpMode {

    private MecanumDrive drive; 
    private IMU imu; 
    private DcMotor leftOut;
    private DcMotor rightOut;
    private DcMotor belt;
    private DcMotor intake;
    private Servo leftRotater;
    private Servo rightRotater;
    private Servo kick; 
    
  

      private double applyDeadzone(double value, double dz) {
        return Math.abs(value) < dz ? 0.0 : value;
    }
    @Override
    public void runOpMode() throws InterruptedException {
  
        // Initialize 
        drive = new MecanumDrive(hardwareMap);
        leftOut = hardwareMap.get(DcMotor.class, "leftOut");
        rightOut = hardwareMap.get(DcMotor.class, "rightOut");
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftRotater = hardwareMap.get(Servo.class, "leftRotater");
        rightRotater = hardwareMap.get(Servo.class, "rightRotater");
        kick = hardwareMap.get(Servo.class, "kick");

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters();
        imuParams.angleUnit = AngleUnit.RADIANS;
        imu.initialize(imuParams);

        waitForStart();

        while (opModeIsActive()) {

            // Get current heading in radians (yaw)
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Joystick inputs
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Deadzone
            y = applyDeadzone(y, 0.05);
            x = applyDeadzone(x, 0.05);
            rx = applyDeadzone(rx, 0.05);

            // Field-centric transform
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;

            // Send to Road Runner drive
            drive.setDrivePower(new Pose2d(rotY, rotX, rx));

            // Update drive
            drive.update();

            // Optional telemetry
            telemetry.addData("Heading (rad)", heading);
            telemetry.addData("Input (Y, X, RX)", "%.2f %.2f %.2f", y, x, rx);
            telemetry.update();

           if (gamepad1.y) {
                imu.resetYaw(); // reset field-centric
            }
          double inPower = gamepad1.right_trigger - gamepad1.left_trigger; // intake
          intake.setPower(inPower);

          //outtake

          if (gamepad1.a) {
            leftOut.setPower(1.0); //may need to reverse either or none at all
            rightOut.setPower(-1.0);
          } 
          else {
            leftOut.setPower(0);
            rightOut.setPower(0);
          }
          if (gamepad1.dpad_up) {
            kick.setPosition(0.8); //kick (tune)
          }
          else if (gamepad1.dpad_down) {
            kick.setPosition(0.3); //reset (tune)
          }
          double leftServoPos = leftRotater.getPosition();
          double rightServoPos = rightRotater.getPosition();
          

         

if (gamepad1.left_bumper) { // move DOWN
    leftServoPos = Math.min(1.0, leftServoPos + 0.2);
    rightServoPos = Math.max(0.0, rightServoPos - 0.2); // mirrored
} else if (gamepad1.right_bumper) { // move UP
    leftServoPos = Math.max(0.0, leftServoPos - 0.2);
    rightServoPos = Math.min(1.0, rightServoPos + 0.2); // mirrored
}

// Update servo positions
leftRotater.setPosition(leftServoPos);
rightRotater.setPosition(rightServoPos);
          
            


          
          

          
          
          
          

            
        }
    }

  
}
