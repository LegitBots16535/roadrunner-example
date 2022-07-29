package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Legitbot;

public class Teleop2P_V1 extends OpMode {
    Legitbot legitbot;
    @Override
    public void init() {
        legitbot = new Legitbot(hardwareMap);
    }

    @Override
    public void loop() {

        //code required for driving

        //sets binds to float values
        float strafe = gamepad1.left_stick_x;
        float turn = gamepad1.right_stick_x;
        float drive = gamepad1.left_stick_y;

        // calculates power and which direction each motor will go;
        float FR = -strafe - drive + turn;
        float FL = -strafe - drive - turn;
        float BR = -strafe - drive + turn;
        float BL = -strafe - drive - turn;


        //calls setPower function
        legitbot.driveTrain.setPower(FL,FR,BL,BR);

    }
}
