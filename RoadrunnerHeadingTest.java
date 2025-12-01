package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

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



@Autonomous(name = "BlueNear12")

public class RoadrunnerHeadingTest extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, startPose);
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        // 0° heading check
                        .strafeTo(new Vector2d(0, 0))
                        .waitSeconds(0.3)

                        // 90°
                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(90))
                        .waitSeconds(0.3)

                        // 180°
                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(180))
                        .waitSeconds(0.3)

                        // 270°
                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(270))
                        .waitSeconds(0.3)

                        // Diagonals (45°, 135°, 225°, 315°)
                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(45))
                        .waitSeconds(0.3)

                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(135))
                        .waitSeconds(0.3)

                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(225))
                        .waitSeconds(0.3)

                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(315))
                        .waitSeconds(0.3)

                        .build()
        );
    }
}
      
