package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/**
 * Created by michaudc on 10/8/2017.
 * Updated by michaudc on 29 July 2021
 * Additional code by Allesio Toniolo July 2021
 * Based on HardwarePushbot Code from the FTCRobotController resources
 * <p>
 * Revision for 2022 Season V1 on 19 Aug 22 by michaudc
 * <p>
 * Revision for 2023 Season V1 on 24 July 22 by michaudc
 * Added Java Karel style method calls for move, turnLeft, turnRight, turn
 * Overloaded Methods with (distance, speed) and (distance) formats
 * <p>
 * Revision for 2024-2025 Season V1 05 Jun 24 by Cole S, Russell M, and michaudc
 * Added methods for field centric drive in Base robot
 * Revision for 2024 Updated getBrightness() to return value from HSV
 * <p>
 * This class models the physical structure of the robot with instances
 * of motors, servos, and sensors.
 * <p>
 * The following are name assignments to be configured
 * on the RC Phone in the App.
 * <p>
 * Motor channel: leftFront:        "leftfront"
 * Motor channel: rightFront:       "rightfront"
 * Motor channel: leftRear:          "leftrear"
 * Motor channel: rightRear:         "rightrear"
 */

public class TBotzRobotDriver {
    /* Public Motors and Servos */
    public final DcMotor leftFront;
    public final DcMotor rightFront;
    public final DcMotor leftRear;
    public final DcMotor rightRear;
    public final IMU imu;

    /**
     *
     * @param hwMap Used to get motors for the wheels. Assign Names that match the setup on the RC Phone
     * @param logoDirection Define for Orientation of REV Control Hub
     * @param usbDirection Define for Orientation of REV Control Hub
     */
    public TBotzRobotDriver(
            HardwareMap hwMap,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection
    ) {
        leftFront = hwMap.get(DcMotor.class, "leftfront");
        rightFront = hwMap.get(DcMotor.class, "rightfront");
        leftRear = hwMap.get(DcMotor.class, "leftrear");
        rightRear = hwMap.get(DcMotor.class, "rightrear");

        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to run without encoders.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // IMU Sensor for Field Centric Driving
        imu = hwMap.get(IMU.class, "imu");

        // Initialize the IMU
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    //added by Cole Saunders 22 Oct 23
    public double getOrientation() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetImu() {
        imu.resetYaw();
    }

    // Updated Mr. Michaud Aug 25
    public void driveFieldCentric(double gamepadXPow, double gamepadYPow, double gamepadRotPow) {
        double robotHeading = getOrientation();

        double gamepadTheta = Math.atan2(gamepadYPow, gamepadXPow) - Math.PI / 2;
        double velocity = Math.sqrt(Math.pow(gamepadXPow, 2) + Math.pow(gamepadYPow, 2));
        double diffTheta = gamepadTheta + Math.toRadians(robotHeading);

        double v1 = velocity * Math.sin(diffTheta + Math.PI / 4) - gamepadRotPow; // leftfront
        double v2 = velocity * Math.cos(diffTheta + Math.PI / 4) + gamepadRotPow; // rightfront
        double v3 = velocity * Math.cos(diffTheta + Math.PI / 4) - gamepadRotPow; // leftrear
        double v4 = velocity * Math.sin(diffTheta + Math.PI / 4) + gamepadRotPow; // rightrear

        leftFront.setPower(v1);
        leftRear.setPower(v3);
        rightFront.setPower(v2);
        rightRear.setPower(v4);
    }

    public void driveStrafer(double leftX, double leftY, double rightX) {
        double leftFrontPower = leftY - leftX - rightX;
        double leftRearPower = leftY + leftX - rightX;
        double rightFrontPower = leftY + leftX + rightX;
        double rightRearPower = leftY - leftX + rightX;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void driveTank(double leftY, double rightY) {
        leftFront.setPower(leftY);
        leftRear.setPower(leftY);
        rightFront.setPower(rightY);
        rightRear.setPower(rightY);
    }
}


