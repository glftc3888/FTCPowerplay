package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class LeoOpMode extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();

    DcMotor topLeft = null;
    DcMotor topRight = null;
    DcMotor bottomLeft = null;
    DcMotor bottomRight = null;

    final double RADIUS=2;
    final double PI=3.1415926535;
    final double CIRCUMFERENCE = 2*PI*RADIUS;
    final int TPR = 1440;

    public void initRobo() {
        //Control Hub
        topLeft  = hardwareMap.get(DcMotor.class, "front_left");
        topRight  = hardwareMap.get(DcMotor.class, "front_right");
        bottomLeft  = hardwareMap.get(DcMotor.class, "back_left");
        bottomRight  = hardwareMap.get(DcMotor.class, "back_right");

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //experimental code with PID control ----------------------------
    public void setColumnPow(double powLeft, double powRight){
        topLeft.setPower(powLeft);
        topRight.setPower(powRight);
        bottomLeft.setPower(powLeft);
        bottomRight.setPower(powRight);
    }

    ///Set Single Power
    public void setSinglePow(double pow){
        topLeft.setPower(pow);
        topRight.setPower(pow);
        bottomLeft.setPower(pow);
        bottomRight.setPower(pow);
    }

    //Set Each Power
    public void setEachPow(double tLpow, double tRpow, double bLpow, double bRpow){
        topLeft.setPower(tLpow);
        topRight.setPower(tRpow);
        bottomLeft.setPower(bLpow);
        bottomRight.setPower(bRpow);
    }


    //Read Motors encoders
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
        telemetry.update();
    }

    //Read Arm encoders

    //Sets all target positions
    public void moveRobot(int left, int right, double power) throws InterruptedException{
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setTargetPosition(left);
        topRight.setTargetPosition(right);
        bottomLeft.setTargetPosition(left);
        bottomRight.setTargetPosition(right);

        topLeft.setPower(power);
        topRight.setPower(power);
        bottomLeft.setPower(power);
        bottomRight.setPower(power);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(topLeft.isBusy() || topRight.isBusy() || bottomLeft.isBusy() || bottomRight.isBusy()) {
            readEncoder();
        }

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void linearY(double inches, double power) throws InterruptedException {
        int ticks = (int)(inches/CIRCUMFERENCE*TPR);
        moveRobot(ticks, ticks, power);
    }
    /*
    public void turnHeading(double angle) throws InterruptedException {
        turnHeading(angle,0.5f);
    }
    public void turnHeading(double angle, double pow) throws InterruptedException {
        double power = pow;
        double m_P = 5.5;
        double tol = 2.5;
        //double pow = 1;

        double err = (angle-this.getAngle());


        while(opModeIsActive() && Math.abs(err)>tol){
            int ticks = (int)(m_P*err);
            moveRobot(-ticks, ticks, power);
            err = (angle-this.getAngle());
            telemetry.addLine(String.valueOf(this.getAngle()));
            telemetry.update();
        }
    }
    public void turnHeadingSmooth(double angle, double pow) throws InterruptedException {
        double power = pow;
        double m_P = 5.5;
        double tol = 2.5;

        double err = (angle-getAngle());

        while(opModeIsActive() && Math.abs(err)>tol){
            int ticks = (int)(m_P*err);
            double sign = err / Math.abs(err);
            this.setColumnPow(-sign*pow, sign*pow);
            err = (angle-getAngle());
            telemetry.addLine(String.valueOf(this.getAngle()));
            telemetry.update();
        }
    }

     */
}