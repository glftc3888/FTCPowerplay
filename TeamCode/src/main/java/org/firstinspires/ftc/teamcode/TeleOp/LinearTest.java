package org.firstinspires.ftc.teamcode.TeleOp;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// code from previous year - encoders, camera, motors
// https://github.com/greasedlightning/FtcRobotController

@TeleOp(name = "TestingLinearSlide", group = "TeleOp")
public class LinearTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LinearSlide = null;

    boolean teleop = false;

    // roughly 537.7, but ((((1+(46/17))) * (1+(46/11))) * 28) to be exact (on the site)
    // Link for motor:
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // Ticks Per Rotation (how many ticks in one full motor rotation)
    private static final double TPR = (1+((double)46/17)) * (1+((double)46/11)) * 28;// ticks per revolution
    // circumference of the pulley circle pulling the string in linear slides
    private static final double CIRCUMFERENCE = 112; // in mm
    // DON'T USE THIS, IF IT'S TOO MUCH IT MIGHT BREAK THE LINEAR SLIDE
    private int MAX_LINEAR_SLIDE_EXTENSION = 4080; // in ticks
    private int ZERO_TICKS_LINEAR_SLIDE = 10; // zero position of linear slides (not 0 due to overextension issues) - in ticks
    private static final double RADIUS = 4.8; // in cm
    private static final double PI=3.1415926535;
    private static final double WHEEL_CIRCUMFERENCE = 2*PI*RADIUS;

    boolean continousMode = false;
    int position = 0;
    //private BNO055IMU Gyro;


    @Override
    public void runOpMode() throws InterruptedException {


        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        teleop = true;

        initRobot();

        // Wait for start
        waitForStart();
        runtime.reset();

        // Start of OpMode
        while (opModeIsActive()) {


            LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            position = LinearSlide.getCurrentPosition();
            LinearSlide.setPower(-gamepad2.left_stick_y/2);
            if (gamepad2.a) {
                setSlideMMAbsolute(350, .6);
            }

            // height 2 (medium junction)
            if (gamepad2.b) {
                setSlideMMAbsolute(595, .6);
            }

            // height 3 (high junction) -- 850mm, but we can't go that much yet
            if (gamepad2.y) {
                setSlideTicksAbsolute(MAX_LINEAR_SLIDE_EXTENSION, .6);
            }

            // down from any position
            if (gamepad2.x) {
                setSlideBottomAbsolute(.6);
            }


        }
    }

    public void initRobot(){
        // Setup hardware


        LinearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");



        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // keep it reverse if you want positive ticks to move linear slide up
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        //when you're setting it up (in opMode)
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LinearSlide.setPower(0);

    }


    // Make sure the encoder cables are connected right, and the the forward/backward is in the right place
    // setSlideTicksAbsolute moves the linear slide to a certain tick POSITION (not BY a certain amount)
    public void setSlideTicksAbsolute(int ticksPosition, double power) throws InterruptedException{
        // move BY difference between the positions the linear slide is at
        // int currentPosition = LinearSlide.getCurrentPosition();
        //int positionDifference = ticksPosition - currentPosition;

        // move by that amount of ticks
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setTargetPosition(ticksPosition);
        LinearSlide.setPower(power);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(LinearSlide.isBusy()) {


        }

        // Uncomment below if you want linear slides to just stop powering once you get to position
        //LinearSlide.setPower(0);
        //LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // set linear slide to certain tick position from MILLIMETER measurement upwards
    public void setSlideMMAbsolute(int mm, double power) throws InterruptedException {
        // convert from MM to ticks
        // first convert from mm to rotations (mm / CIRCUMFERENCE) = rotations
        // then convert from rotations to ticks ( rotations * TPR
        int ticksFromMM = (int)( (mm / CIRCUMFERENCE) * TPR);
        setSlideTicksAbsolute(ticksFromMM, power);
    }
    public void setSlideBottomAbsolute(double power) throws InterruptedException {
        setSlideTicksAbsolute(ZERO_TICKS_LINEAR_SLIDE, power);
        LinearSlide.setPower(0);
    }


    public void setSlideMaxAbsolute(double power) throws InterruptedException {
        setSlideTicksAbsolute(MAX_LINEAR_SLIDE_EXTENSION, power);
    }


    public void setPowerLinearSlide(double power){
        LinearSlide.setPower(power);
    }

    /*
    // move servo for certain amount of (milliseconds) with (power)
    public void moveServo(long ms, double power) throws InterruptedException {
        setPowerServo(power);
        Thread.sleep(ms);
        setPowerServo(0);
    }

     */
}

