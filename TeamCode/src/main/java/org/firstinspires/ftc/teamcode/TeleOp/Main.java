package org.firstinspires.ftc.teamcode.TeleOp;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    private static CRServo leftServo = null;
    private static CRServo rightServo = null;

    private DcMotor LinearSlide = null;

    boolean teleop = false;

    // will use later... This is for angle (see code from last year)
    BNO055IMU IMU;

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

            // move mecanum drivetrain using gamepad values
            if (gamepad1.left_trigger > .6f) {
                setPowerMecanumGamepad(.25);
            } else {
                setPowerMecanumGamepad(.5);
            }


            // controlling linear slides and intake -- gamepad 2

            //servo intake, servo outtake
            if(gamepad2.left_trigger > .6f) { setPowerServo(1);}
            if(gamepad2.right_trigger > .6f) { setPowerServo(-1);}
            // default to not having servo move (only move when triggered)
            setPowerServo(0);

            // state variables (switching between them)
            // NO LOCKING
            // if (gamepad2.left_stick_button) {lock = !lock;}

            if (gamepad2.right_bumper) { continousMode = true; }
            if (gamepad2.left_bumper) { continousMode = false; }

            telemetry.addLine(String.valueOf(continousMode));
            telemetry.update();

            // do things depending on states
            if (continousMode) {
                //telemetry.addLine("This telemetry is crucial for the structural integrity of this code.");
                //telemetry.update();
                LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                position = LinearSlide.getCurrentPosition();
                LinearSlide.setPower(-gamepad2.left_stick_y/2);
            }
            else {
                //telemetry.addLine("non cont. mode");
                //telemetry.update();
                //linear slides
                // height 1 (low junction)
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
    }

    public void initRobot(){
        // Setup hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        LinearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");

        leftServo = hardwareMap.crservo.get("left_servo");                 //left CR Servo
        rightServo = hardwareMap.crservo.get("right_servo");                 //right CR Servo

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        LinearSlide.setPower(0);
    }

    public void setPowerMecanumGamepad(double constant){
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        frontLeftMotor.setPower((leftY - leftX - rightX) * constant);
        frontRightMotor.setPower((leftY + leftX + rightX) * constant);
        backRightMotor.setPower((leftY - leftX + rightX)* constant);
        backLeftMotor.setPower((leftY + leftX - rightX)* constant);
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
            if (teleop) {
                if (gamepad1.left_trigger > .6f) {
                    setPowerMecanumGamepad(.25);
                } else {
                    setPowerMecanumGamepad(.5);
                }

            }
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

    public boolean isBusy() {
        return (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy() || frontRightMotor.isBusy());
    }

    public void forward(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
    }

    public  void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public  void turn(double power) {
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
    }

    public  void encoderForward(double cm, double power) {
        int ticks = (int) ((cm / WHEEL_CIRCUMFERENCE) * TPR);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forward(power);

        frontLeftMotor.setTargetPosition(-ticks);
        frontRightMotor.setTargetPosition(-ticks);
        backLeftMotor.setTargetPosition(-ticks);
        backRightMotor.setTargetPosition(-ticks);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(isBusy()){ readEncoder(); };

        forward(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void encoderStrafe(double cm, double power) {
        // Fix the straffing encoders
        // 1) It has to work
        // 2) Since it's straffing it will move less than inches

        double EXPERIMENTAL_CIRCUMFERENCE = 27.75;
        //int ticks = (int) ((cm / WHEEL_CIRCUMFERENCE) * TPR);
        int ticks = (int) ((cm / EXPERIMENTAL_CIRCUMFERENCE) * TPR);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forward(power);

        frontLeftMotor.setTargetPosition(-ticks);
        frontRightMotor.setTargetPosition(ticks);
        backLeftMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(-ticks);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(isBusy()){}

        forward(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", frontRightMotor.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", backLeftMotor.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", backRightMotor.getCurrentPosition());
        telemetry.update();
    }

    // set power of servo to (power)
    public void setPowerServo(double power) {
        leftServo.setPower(-power);
        rightServo.setPower(power);
    }

    public void setPowerLinearSlide(double power){
        LinearSlide.setPower(power);
    }

    // move servo for certain amount of (milliseconds) with (power)
    public void moveServo(long ms, double power) throws InterruptedException {
        setPowerServo(power);
        Thread.sleep(ms);
        setPowerServo(0);
    }
}

