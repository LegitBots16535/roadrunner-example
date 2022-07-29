package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {
    private final BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    public IMU(HardwareMap hardwareMap){imu = hardwareMap.get(BNO055IMU.class, "imu");}

    public void initIMU(){
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode                = BNO055IMU.SensorMode.IMU;
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled      = false;
        imu.initialize(params);

        while (!imu.isGyroCalibrated())
        {
            // Calibrates Gyro
        }
    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle()
    {
        // Returns us the current angle from lastAngles
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public double checkDirection()
    {
        // Our P Constant to find the adjusted angle needed
        double correction, angle; //here gain is the kP
        double kP = 0.00005;

        angle = getAngle();
        correction = -angle;
        /*
        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.
         */
        correction = correction * kP;
        return correction;
    }

    public double checkDirection(double targDegrees)
    {
        // Our P Constant to find the adjusted angle needed
        double correction, angle; //here gain is the kP
        double kP = 0.00005;

        angle = getAngle();
        correction = targDegrees - angle;
        /*
        if (angle == targDegrees)
            correction = 0;             // no adjustment.
        else
            correction = targDegrees-angle;        // reverse sign of angle for correction
         */
        correction = correction * kP;
        return correction;
    }

}