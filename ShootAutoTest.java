/**
 * Author: Anthony Kongoasa
 * Gets into position and Shoots 3 artifacts 
 */
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectorysequence.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.Actions;

@Autonomous(name = "Shoot Auto", group = "Auto")
public class ShootAutoTest extends LinearOpMode {
    private DcMotor outtake, belt, intake, tilt;
    private Servo kick;
    private MecanumDrive drive;
    

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        drive   = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt    = hardwareMap.get(DcMotor.class, "belt");
        kick    = hardwareMap.get(Servo.class, "kick");
        tilt    = hardwareMap.get(Servo.class, "tilt");

        waitForStart();
        if (isStopRequested()) return;
        belt.setPower(0.6);
       Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .linetoX(5)
                            .turn(Math.toRadians(50))
                            .build();
            );
        
        // Shoot 3
        shoot();

/**
     Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(20, -20), Math.toRadians(90))
                            .build();
            );
     intake.setPower(1);
     Actions.runBlocking( //intake pile
                    drive.actionBuilder(drive.pose)
                            .lineToY(-30)
                            .splineTo(new Vector2d(5, 0), Math.toRadians(50))
                            .build();
            );
    intake.setPower(0);
    shoot();
  

            
*/      
belt.setPower(0);
        

        telemetry.addLine("Auto finished");
        telemetry.update();
        sleep(2000);
    }

    private void shoot() throws InterruptedException {
    // Spin up shooter
    outtake.setPower(1.0);
    sleep(2500); // allow shooter to get to speed
    belt.setPower(1); // fast to push the artifact in
     kick.setPosition(0.8); //open
    sleep(2500);
    kick.setPosition(0.3); //close
    belt.setPower(0.6);
    // Turn off shooter after firing
    outtake.setPower(0);
  }
}
