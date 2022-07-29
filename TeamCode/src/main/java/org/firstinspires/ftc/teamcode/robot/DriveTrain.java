package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.IMU;

public class DriveTrain {

    public enum DriveState{
        IDLE,
        TELE,
        DRIVING,
        STRAFING,
        TURNING_L,
        TURNING_R
    }

    private DcMotor bl, br, fl, fr;
    private IMU imu;
    private DriveState state;
    private double currentTargDeg;
    private final double rotPerInch = 0.0795774715459477;
    private final double ticksPerRev = 537.6;
    private final double degPerRot = 1;//75
    private final double sensitivity = 0.7;


    public DriveTrain(HardwareMap hardwareMap, IMU robotIMU){
        //Initialize Motors
        bl = hardwareMap.get(DcMotor.class, "LB");
        br = hardwareMap.get(DcMotor.class, "RB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Add IMU and set initial state to IDLE
        this.imu = robotIMU;
        this.imu.initIMU();
        this.state = DriveState.IDLE;
    }

    public void setBrake(){
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Moves all drive train motors forward or backward

    public void setPower(float FL, float FR,float BL,float  BR) {
        fl.setPower(FL* sensitivity);
        fr.setPower(FR * sensitivity);
        bl.setPower(BL * sensitivity);
        br.setPower(BR * sensitivity);
        setState(DriveState.DRIVING);


    }
    public void MoveTank(int inches, double speed){
        imu.resetAngle();
        double rotations =(double) inches * rotPerInch;
        SetTargPos((int) (rotations * ticksPerRev));
        setState(DriveState.DRIVING);
        bl.setPower(speed);
        br.setPower(speed);
        fl.setPower(speed);
        fr.setPower(speed);
    }

    // Strafing
    public void strafeLeft(int inches, double speed){
        double rotations = (double) inches * rotPerInch;
        SetTargPos((int) (rotations * ticksPerRev));
        br.setTargetPosition(-br.getTargetPosition());
        fl.setTargetPosition(-fl.getTargetPosition());
        setState(DriveState.STRAFING);
        bl.setPower(speed);
        br.setPower(-speed);
        fl.setPower(-speed);
        fr.setPower(speed);
    }

    public void strafeRight(int inches, double speed){
        double rotations = (double) inches * rotPerInch;
        SetTargPos((int) (rotations * ticksPerRev));
        bl.setTargetPosition(-bl.getTargetPosition());
        fr.setTargetPosition(-fr.getTargetPosition());
        setState(DriveState.STRAFING);
        bl.setPower(-speed);
        br.setPower(speed);
        fl.setPower(speed);
        fr.setPower(-speed);
    }

    // Turning

    public void turnLeft(int degrees, double speed){
        imu.resetAngle();
        currentTargDeg = degrees;
        SetTargPos((int) (((double) degrees/degPerRot) * ticksPerRev));
        bl.setTargetPosition(-bl.getTargetPosition());
        fl.setTargetPosition(-fl.getTargetPosition());
        setState(DriveState.TURNING_L);
        bl.setPower(-speed);
        br.setPower(speed);
        fl.setPower(-speed);
        fr.setPower(speed);
    }

    public void turnRight(int degrees, double speed){
        imu.resetAngle();
        currentTargDeg = -degrees;
        SetTargPos((int) (((double) degrees/degPerRot) * ticksPerRev));
        br.setTargetPosition(-br.getTargetPosition());
        fr.setTargetPosition(-fr.getTargetPosition());
        setState(DriveState.TURNING_R);
        bl.setPower(speed);
        br.setPower(-speed);
        fl.setPower(speed);
        fr.setPower(-speed);
    }

    // IMU BASED CALCULATION
    private void adjustPow(double correction){
        fr.setPower(fr.getPower() + correction);
        br.setPower(br.getPower() + correction);
        fl.setPower(fl.getPower() - correction);
        bl.setPower(bl.getPower() - correction);


    }

    private void adjustAllPow(double inc)
    {
        if(br.getPower() < 0){
            br.setPower(br.getPower() + inc);
        }
        else{
            br.setPower(br.getPower() - inc);
        }

        if(fr.getPower() < 0){
            fr.setPower(fr.getPower() + inc);
        }
        else{
            fr.setPower(fr.getPower() - inc);
        }

        if(fl.getPower() < 0){
            fl.setPower(fl.getPower() + inc);
        }
        else{
            fl.setPower(fl.getPower() - inc);
        }

        if(bl.getPower() < 0){
            bl.setPower(bl.getPower() + inc);
        }
        else{
            bl.setPower(bl.getPower() - inc);
        }
    }

    public void addLeftPow(double inc){
        bl.setPower(br.getPower() + inc);
        fl.setPower(fl.getPower() + inc);
    }

    private void rampPow()
    {
        double currentDelta;
        double minPower = 0.1;
        if(Math.abs(getFrPow()) > minPower)
        {
            if(state == DriveState.DRIVING)
            {
                currentDelta = Math.abs(getTargetPos() - getCurrentPos());
                if (currentDelta < 0.6 * Math.abs(getTargetPos()))
                {
                    adjustAllPow(0.02);
                }
            }
            else if (state == DriveState.TURNING_L ||
                    state == DriveState.TURNING_R)
            {
                currentDelta = Math.abs(imu.getAngle() - currentTargDeg);
                if(currentDelta < 0.6 * Math.abs(currentTargDeg))
                {
                    adjustAllPow(0.07);
                }
            }
        }
    }

    public void StopMotors(){
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        setState(DriveState.IDLE);
        resetMotors();
    }
    public void SetTargPos(int targ)
    {
        StopMotors();
        bl.setTargetPosition(targ);
        br.setTargetPosition(targ);
        fl.setTargetPosition(targ);
        fr.setTargetPosition(targ);

        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update(){
        double correction = 0;
        switch (this.state){
            case DRIVING:
                if(isTargetReached()){
                    setState(DriveState.IDLE);
                    break;
                }
                correction = imu.checkDirection();
                adjustPow(correction);
                rampPow();
                break;
            case STRAFING:
                if(getCurrentPos() == getTargetPos()){
                    setState(DriveState.IDLE);
                    break;
                }
                break;
            case TURNING_L:
                rampPow();
                if(imu.getAngle() >= currentTargDeg && imu.getAngle() != 0){
                    setState(DriveState.IDLE);
                    break;

                }
                //correction = imu.checkDirection(currentTargDeg * 1.1);
                //adjustPow(correction);

                break;
            case TURNING_R:
                rampPow();
                if(imu.getAngle() <= currentTargDeg && imu.getAngle() != 0){
                    setState(DriveState.IDLE);
                    break;
                }
                //correction = imu.checkDirection(currentTargDeg * 1.1);
                //adjustPow(correction);
                break;
            case IDLE:
                StopMotors();
                break;
            case TELE:
                break;
        }
    }

    // TeleOP --------------------------------------------------

    //Tele Move Tank
    public void TeleMoveTank(double leftPow, double rightPow){
        setState(DriveState.TELE);
        setTeleOp();
        fl.setPower(leftPow);
        fr.setPower(rightPow);
        bl.setPower(leftPow);
        br.setPower(rightPow);
    }

    // Tele strafing
    public void teleStrafeLeft(double pow){
        setState(DriveState.TELE);
        setTeleOp();
        fl.setPower(-pow);
        fr.setPower(pow);
        bl.setPower(pow);
        br.setPower(-pow);
    }

    public void teleStrafeRight(double pow){
        setState(DriveState.TELE);
        setTeleOp();
        fl.setPower(pow);
        fr.setPower(-pow);
        bl.setPower(-pow);
        br.setPower(pow);
    }

    // Reset the motors
    public void resetMotors(){
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Setting TeleOp Mode
    public void setTeleOp(){
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Setting State
    public void setState(DriveState state){this.state = state;}
    public boolean isTargetReached(){
        return br.getCurrentPosition() == br.getTargetPosition() || fl.getCurrentPosition() == fl.getTargetPosition() || bl.getCurrentPosition() == bl.getTargetPosition() || fr.getCurrentPosition() == fr.getTargetPosition();
    }

    // Getters
    public double getFrPow(){return fr.getPower();}
    public int getTargetPos(){return br.getTargetPosition();}
    public int getCurrentPos(){return br.getCurrentPosition();}
    public DriveState getState(){return state;}
}
