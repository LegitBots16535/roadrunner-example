package org.firstinspires.ftc.teamcode.auto.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class test_traj extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

            Trajectory traj1 = robot.trajectoryBuilder(new Pose2d(0,0), true)
                .splineTo(new Vector2d(23, -36), Math.toRadians(180))
                .build();
    }
}
