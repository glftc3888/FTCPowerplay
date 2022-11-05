package org.firstinspires.ftc.teamcode.Autonomous;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

// code from previous year - encoders, camera, motors
// https://github.com/greasedlightning/FtcRobotController
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    // Declaration of global variables
    private ElapsedTime runtime = new ElapsedTime();

    private static DcMotor frontLeftMotor = null;
    private static DcMotor backLeftMotor = null;
    private static DcMotor frontRightMotor = null;
    private static DcMotor backRightMotor = null;

    private DcMotor LinearSlide = null;

    // will use later... This is for angle (see code from last year)
    BNO055IMU IMU;

    // roughly 537.7, but ((((1+(46/17))) * (1+(46/11))) * 28) to be exact (on the site)
    // Link for motor:
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // Ticks Per Rotation (how many ticks in one full motor rotation)
    private static final double TPR = (1+(46/17)) * (1+(46/11)) * 28;// ticks per revolution
    // circumference of the pulley circle pulling the string in linear slides
    private static final double CIRCUMFERENCE = 112; // in mm
    // DON'T USE THIS, IF IT'S TOO MUCH IT MIGHT BREAK THE LINEAR SLIDE
    private double MAX_LINEAR_SLIDE_EXTENSION = 976; // in mm
    private static final int ticksToWheelRevolution = 1440;
    private static final double RADIUS= 3.5;
    private static final double PI=3.1415926535;
    private static final double circum = 2*PI*RADIUS;

    @Override
    public void runOpMode() throws InterruptedException {
        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        // Setup hardware
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "back_left");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right");

        //LinearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");

        //LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        //LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        //LinearSlide.setPower(0);

        // Wait for start
        waitForStart();
        runtime.reset();


        // Start of OpMode
        while (opModeIsActive()) {
            encoderForward(4);
            readEncoder();
            stopRobot();

            //int pos = LinearSlide.getCurrentPosition();
            //telemetry.addLine(String.valueOf(pos));
            //telemetry.update();
            telemetry.addLine("your mom");
            telemetry.update();
            // linear slide code testing



        }
    }


    // Make sure the encoder cables are connected right, and the the forward/backward is in the right place
    // setSlideTicksAbsolute moves the linear slide to a certain tick POSITION (not BY a certain amount)
    /*
    public void setSlideTicksAbsolute(int ticksPosition, double power) throws InterruptedException{
        // move BY difference between the positions the linear slide is at
        int currentPosition = LinearSlide.getCurrentPosition();
        int positionDifference = ticksPosition - currentPosition;

        // move by that amount of ticks
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setTargetPosition(positionDifference);
        LinearSlide.setPower(power);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(LinearSlide.isBusy()) {
        }

        // Uncomment below if you want linear slides to just stop powering once you get to position
        //linearSlide.setPower(0);
        //linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
     */

    // set linear slide to certain tick position from MILLIMETER measurement upwards
    /*
    public void setSlideMMAbsolute(int mm, double power) throws InterruptedException {
        // convert from MM to ticks
        // first convert from mm to rotations (mm / CIRCUMFERENCE) = rotations
        // then convert from rotations to ticks ( rotations * TPR)
        int ticksFromMM = (int)( (mm / CIRCUMFERENCE) * TPR);
        setSlideTicksAbsolute(ticksFromMM, power);
    }

     */
    public static boolean isMoving() {
        return (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy() || frontRightMotor.isBusy());
    }

    public void forward(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
    }

    public static void strafe(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
    }

    public void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public static void turn(double power) {
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
    }

    public void encoderForward(double inches) {
        //int ticks = (int) ((inches / circum) * ticksToWheelRevolution);
        int ticks=5000;
        forward(0.75);

        telemetry.addLine("doing stuff");
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setTargetPosition(-ticks);
        frontRightMotor.setTargetPosition(-ticks);
        backLeftMotor.setTargetPosition(-ticks);
        backRightMotor.setTargetPosition(-ticks);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        forward(0);
        stopRobot();



    }

    public static void encoderStrafe(double inches) {
        int ticks = (int) (inches / circum) * ticksToWheelRevolution;

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setTargetPosition(-ticks);
        frontRightMotor.setTargetPosition(-ticks);
        backLeftMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(ticks);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        strafe(0.75);

    }
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", frontRightMotor.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", backLeftMotor.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", backRightMotor.getCurrentPosition());
        telemetry.update();
    }
}





