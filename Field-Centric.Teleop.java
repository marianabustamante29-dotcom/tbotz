 /** for roadrunner/android studio
 assuming we use the intake - belt - outtake robot system
 @author Anthony Kongoasa
    */
// NEED TO CHANGE
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;  
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Field Centric TeleOp")
public class FieldCentricTeleOpRR_ResetYaw extends LinearOpMode {

    private MecanumDrive drive; 
    private IMU imu; 
    private DcMotor outtake;
    private DcMotor tilt;
    private DcMotor belt;
    private DcMotor intake;
  
    private Servo kick; 
    
  

      private double applyDeadzone(double value, double dz) {
        return Math.abs(value) < dz ? 0.0 : value;
    }
    @Override
    public void runOpMode() throws InterruptedException {
  
        // Initialize 
        drive = new MecanumDrive(hardwareMap);
        outtake = hardwareMap.get(DcMotor.class, "outtake");
       
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");
        tilt = hardwareMap.get(DcMotor.class, "tilt");
        kick = hardwareMap.get(Servo.class, "kick");
        
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters();
        imuParams.angleUnit = AngleUnit.RADIANS;
        imu.initialize(imuParams);
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
          double inPower = gamepad2.right_trigger - gamepad2.left_trigger; // intake
          intake.setPower(inPower);
         double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        // Limit Power to -0.4 to 0.4
        if (armMotorPower > 0.1) {
            ARM_SPEED = 0.7;
            armPos += gamepad1.right_trigger * 10;
        }

        if (armMotorPower < -0.1) {
            ARM_SPEED = 0.7;
            armPos -= gamepad1.left_trigger * 10;
        }
        tilt.setTargetPosition(armPos);
        ///tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setPower(ARM_SPEED);
          //outtake
         
          if (gamepad1.a) {
            outtake.setPower(1.0); //may need to reverse  
            
          } 
          else {
            outtake.setPower(0);
          }
          if (gamepad1.dpad_up) {
            kick.setPosition(0.8); //kick (tune)
          }
          else if (gamepad1.dpad_down) {
            kick.setPosition(0.3); //reset (tune)
          }
       
          
//tun CAREFULLY   



          
          

          
          
          
          

            
        }
    }

  
}
