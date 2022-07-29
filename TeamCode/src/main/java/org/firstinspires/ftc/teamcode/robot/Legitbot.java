package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.robot.IMU;


public class Legitbot {

    public IMU imu;

    public DriveTrain driveTrain;
    public Intake intake;


    public Legitbot(HardwareMap hardwareMap){
        imu = new IMU(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, imu);

    }

    // Update Cycle
    public void update(){
        driveTrain.update();
    }
}