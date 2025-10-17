/*
Starter Code for Quad Training Robot 2024
Modified by michaudc 2017, 2023, 2024
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MaristBot2025: Quad Training 2025", group="Training")
//@Disabled
public class TeleopQuad_Training_2025 extends OpMode {

    // Create instance of MaristBaseRobot2024
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad(); 
    DcMotor shooter;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    
    double SHOOT_SPEED_FULL = -3000;
    double SHOOT_SPEED_PART = -2250;
    double SHOOTER_VELOCITY = 0;
    double SPEED_CONTROL = 1.0;
     
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        shooter = robot.leftArm;

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
        if (gamepad1.y) {
            robot.imu.resetYaw();
        }
        double yaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
        double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
        double rightX = gamepad1.right_stick_x * SPEED_CONTROL;
        
        robot.driveFieldCentric(leftX, leftY, rightX, yaw);
        
        //shooter
        if (gamepad1.dpad_left) {
            SHOOTER_VELOCITY = -1390;
        }
        else if (gamepad1.dpad_right) {
            SHOOTER_VELOCITY = SHOOT_SPEED_PART;
        }
        else {
            SHOOTER_VELOCITY = 0;
        }
        robot.leftArm.setVelocity(SHOOTER_VELOCITY);
        robot.rightArm.setVelocity(-SHOOTER_VELOCITY);
        
        // intake
        if (gamepad1.a) {
            robot.intake.setPower(-1);
            robot.uptake.setPower(.5);
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
        
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
