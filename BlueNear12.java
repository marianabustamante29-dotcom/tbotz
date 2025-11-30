package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueNear12")
public class BlueNear12 extends LinearOpMode {
  double ON = -1;
  double OFF = 0;
  double IN = 0.15;
  double NEARVEL = 700; //(tune)

  //PID
  private double NEW_P = 120;
  private double NEW_I = 0;
  private double NEW_D = .8;
  
  DcMotorEx leftOut;
  DcMotorEx rightOut;
  DcMotor intake;
  Servo leftBelt;
  Servo rightBelt;
  Servo leftIndex;
  Servo rightIndex;
  private MecanumDrive drive;

  private ElapsedTime runtime = new ElapsedTime();
  //TODO put PID!!
  
    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware mapping
        leftOut = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightOut = hardwareMap.get(DcMotorEx.class, "rightArm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftBelt = hardwareMap.get(CRServo.class, "leftHand");
        rightBelt = hardwareMap.get(CRServo.class, "rightHand");
        leftIndex = hardwareMap.get(CRServo.class, "leftUptake");
        rightIndex = hardwareMap.get(CRServo.class, "rightUptake");

        //PID
        PIDCoefficients pidSettings = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        leftOut.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidSettings);
        rightOut.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidSettings);

        // Initialize drive
        Pose2d startPose = new Pose2d(-40, 61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        drive.setPoseEstimate(startPose);

        // Transformed shoot position
        Vector2d shootPos = new Vector2d(34, 34);
  
        waitForStart();
        if (isStopRequested()) return;
  
        leftOut.setVelocity(-NEARVEL);
        rightOut.setVelocity(NEARVEL);

        // -------- Shoot #1 ----------
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(shootPos, Math.toRadians(135), new TranslationalVelConstraint(60))
                        .build()
        );
        shoot();
      
        // -------- Intake 1st pile & Shoot #2 ----------
        Actions.runBlocking(
                drive.actionBuilder(drive.getPose())
                        .splineToLinearHeading(new Pose2d(30, 12, Math.toRadians(0)), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(48, 12), Math.toRadians(0), new TranslationalVelConstraint(60))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(50, 6), Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(52, 6), new TranslationalVelConstraint(65))
                        .waitSeconds(0.3)
                        .strafeToLinearHeading(shootPos, Math.toRadians(135), new TranslationalVelConstraint(60))
                        .build()
        );
        shoot();

        // -------- Intake 2nd pile & Shoot #3 ----------
        Actions.runBlocking(
                drive.actionBuilder(drive.getPose())
                        .strafeToLinearHeading(new Vector2d(-30, -11.5), Math.toRadians(0), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(-48, -11.5), Math.toRadians(0))
                        .waitSeconds(0.3)
                        .strafeToLinearHeading(shootPos, Math.toRadians(135), new TranslationalVelConstraint(60))
                        .build()
        );
        shoot();

        // -------- Intake 3rd pile & Final Shoot(4) ----------
        Actions.runBlocking(
                drive.actionBuilder(drive.getPose())
                        .strafeToLinearHeading(new Vector2d(-30, 35), Math.toRadians(0), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(-48, 35), Math.toRadians(0))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(20, -50), Math.toRadians(75), new TranslationalVelConstraint(60))
                        .build()
        );
        shoot();

        // Stop intake/belts
        intake.setPower(OFF);
        rightBelt.setPower(OFF);
        leftBelt.setPower(OFF);

        // Rotate heading for TeleOp setup
        Actions.runBlocking(
                drive.actionBuilder(drive.getPose())
                        .setTangent(Math.toRadians(-90))
                        .build()
        );
    }

    // -------- Shooting helper method ----------
    private void shoot() {
        
        intake.setPower(-ON);
        leftBelt.setPower(-ON);
        rightBelt.setPower(ON);
        leftIndex.setPower(ON);
        rightIndex.setPower(-ON);

        delay(3.0); // wait 3 seconds for shooting

        // Stop all
        intake.setPower(OFF);
        leftBelt.setPower(OFF);
        rightBelt.setPower(OFF);
        leftIndex.setPower(IN);
        rightIndex.setPower(-IN);
    }

    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry
        }
    }
}
