package org.firstinspires.ftc.teamcode.TeleOp;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

// code from previous year - encoders, camera, motors
// https://github.com/greasedlightning/FtcRobotController

@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class Main extends LinearOpMode {

    // Declaration of global variables
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor LinearSlide = null;

    // will use later... This is for angle (see code from last year)
    BNO055IMU IMU;

    // roughly 537.7, but ((((1+(46/17))) * (1+(46/11))) * 28) to be exact (on the site)
    // Link for motor:
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // Ticks Per Rotation (how many ticks in one full motor rotation)
    private double TPR = ((((1+(46/17))) * (1+(46/11))) * 28); // ticks per revolution
    // circumference of the pulley circle pulling the string in linear slides
    private double CIRCUMFERENCE = 112; // in mm
    // DON'T USE THIS, IF IT'S TOO MUCH IT MIGHT BREAK THE LINEAR SLIDE
    private double MAX_LINEAR_SLIDE_EXTENSION = 976; // in mm

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

        LinearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");

        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // keep it reverse if you want positive ticks to move linear slide up
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        //when you're setting it up (in opMode)
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);

        // Wait for start
        waitForStart();
        runtime.reset();

        // Start of OpMode
        while (opModeIsActive()) {
            // Power up the motors using left and right sticks
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            double rightY = gamepad1.right_stick_y;

            // move mecanum drivetrain
            frontLeftMotor.setPower(leftY - leftX - rightX);
            frontRightMotor.setPower(leftY + leftX + rightX);
            backRightMotor.setPower(leftY - leftX + rightX);
            backLeftMotor.setPower(leftY + leftX - rightX);
            telemetry.addData("Status", "in");
            int pos = LinearSlide.getCurrentPosition();
            telemetry.addLine(String.valueOf(pos));
            telemetry.update();

            // linear slide code testing
            /*
            if (gamepad1.a) {
                setSlideMMAbsolute(500, .75);
            }
            if (gamepad1.b) {
                setSlideMMAbsolute(0, .5);
            }*/
        }
    }


    // Make sure the encoder cables are connected right, and the the forward/backward is in the right place
    // setSlideTicksAbsolute moves the linear slide to a certain tick POSITION (not BY a certain amount)
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

    // set linear slide to certain tick position from MILLIMETER measurement upwards
    public void setSlideMMAbsolute(int mm, double power) throws InterruptedException {
        // convert from MM to ticks
        // first convert from mm to rotations (mm / CIRCUMFERENCE) = rotations
        // then convert from rotations to ticks ( rotations * TPR)
        int ticksFromMM = (int)( (mm / CIRCUMFERENCE) * TPR);
        setSlideTicksAbsolute(ticksFromMM, power);
    }
}
