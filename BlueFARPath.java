/**
 * Author: Anthony Kongoasa
 * Gets into position and intakes 8-9, shoots
 */
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectorysequence.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.Actions;

@Autonomous
public class BluefarPath extends LinearOpMode {
    private DcMotor intake; // add others 
     // add servos
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        drive   = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        // change if wanted/needed
        //  drive = new MecanumDrive(hardwareMap, new Pose2d(61,-20,Math.toRadians(180));
        //something like that
        intake = hardwareMap.get(DcMotor.class, "intake");

       //servos init HERE
        
        waitForStart();
        if (isStopRequested()) return;
       //------line up and shoot  --------
        Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .linetoX(-5)
                            .turn(Math.toRadians(50)) //tune
                            .waitSeconds(3) //simulate shoot delay
                            .build()
            ); 
     
      //  shoot();

    //-----------intake from Human player (likely only 2 artifacts) -------------
    intake.setPower(-1);
    Actions.runBlocking(
                  drive.actionBuilder(drive.getPose())
                            .lineToLinearHeading(new Pose2d(-15, 60, Math.toRadians(90)))
                             //tune
                            .waitSeconds(0.5) //delay
                            .lineToX(65)
                            .waitSeconds(0.5)
                            .build()
      );
      
     intake.setPower(0);

      //-------------------------------return to SHOOT pose--------------------------
        Actions.runBlocking(
                  drive.actionBuilder(drive.getPose())
                            .lineToLinearHeading(new Pose2d(-5, 0, Math.toRadians(180)))
                             //tune
                            .waitSeconds(3) //delay
                            .build()
        );
      //shoot();
      // ------------------------intake far line of artifacts--------------
     Actions.runBlocking(
                    drive.actionBuilder(drive.getPose())
                            .splineTo(new Vector2d(-50, 20), Math.toRadians(90))
                            .build()
            );
     intake.setPower(-1); // intake
     Actions.runBlocking( 
                    drive.actionBuilder(drive.getPose())
                            .lineToY(-60)
                            .waitSeconds(0.5)
                            
            );
    intake.setPower(0);
     //-------------------------------return to SHOOT pose--------------------------
        Actions.runBlocking(
                  drive.actionBuilder(drive.getPose())
                            .lineToLinearHeading(new Pose2d(-5, 0, Math.toRadians(180))
                             //tune
                            .waitSeconds(3) //delay
                            .build()
        );
      // shoot();
      Actions.runBlocking(
                drive.actionBuilder(drive.getPose())
                            .lineToY(-10) //get off line
                            .build()

        );

        sleep(2000);
    }

  //  private void shoot() throws InterruptedException {


  
}
