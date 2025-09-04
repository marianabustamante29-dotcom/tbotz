/*
Starter Code for Quad Training Robot 2024
Modified by michaudc 2017, 2023, 2024
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MaristBot2024: Quad Training 2024", group="Training")
//@Disabled
public class TeleopQuad_Training_2024 extends OpMode {

    // Create instance of MaristBaseRobot2024
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad(); 

    
    double SPEED_CONTROL = 1;
    
    private int armPos = 0;
    private int sliderPos = 0;
    
    private double ARM_SPEED = 0.8;
    private double SLIDER_SPEED = 0.8;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        double leftFrontPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        
        double rightFrontPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        
        double leftRearPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        
        double rightRearPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        
        robot.leftFront.setPower(leftFrontPower);
        
        robot.rightFront.setPower(rightFrontPower);
        
        robot.leftRear.setPower(leftRearPower);
        
        robot.rightRear.setPower(rightRearPower);
        
        // Arm Code
        double deltaArmPos = gamepad2.left_stick_y;
        
        if (Math.abs(deltaArmPos) > 0.1) { // Arm is moving
        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setPower(deltaArmPos);
        armPos = robot.leftArm. getCurrentPosition();
            
        }
        else { // Arm is holding Position
            robot.leftArm.setTargetPosition(armPos);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(ARM_SPEED);
        }
        
        telemetry.addData("ArmPos", armPos);
        telemetry.update();
        
        // Slider Code
        double deltaSliderPos = gamepad2.right_stick_y;    
        
        if(Math.abs(deltaSliderPos) > 0.1) { //Slider is moving
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(deltaSliderPos);
            sliderPos = robot.rightArm.getCurrentPosition();

        
            
        }
        else {// Arm is holding Position
            robot.rightArm.setTargetPosition(sliderPos);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setPower(ARM_SPEED);
        }    
    }
    



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        
    }

}
