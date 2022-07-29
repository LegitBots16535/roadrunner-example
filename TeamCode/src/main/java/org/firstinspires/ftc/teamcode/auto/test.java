package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.trajectories.test_traj;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Legitbot;

public class test extends LinearOpMode {
    Legitbot attachments;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




    @Override
    public void runOpMode() throws InterruptedException {

        drive.followTrajectory(traj1);

    }
}
