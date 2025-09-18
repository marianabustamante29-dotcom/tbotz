/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
Modifications for CENTERSTAGE by michaudc 2023
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TBotzTeleopBasic", group = "Training")
//@Disabled
public class TBotzTeleopBasic extends OpMode {

    /* Declare OpMode members. */
    TBotzRobotDriver robot = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Initialize the hardware variables.
        robot = new TBotzRobotDriver(hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");
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
        //The left joystick moves the robot
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
        //The right joystick rotates the robot
        double rightX = gamepad1.right_stick_x;

        robot.driveFieldCentric(leftX, leftY, rightX);

        if (gamepad1.dpad_up) {
            //if dpad "up" is pressed, reset the yaw angle in the gyroscope to reset the field centric direction.
            robot.resetImu();
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


