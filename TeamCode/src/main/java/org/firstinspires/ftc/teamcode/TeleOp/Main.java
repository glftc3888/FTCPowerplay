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

// code from previous year - encoders, camera, motors (we better this year.)
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
    BNO055IMU Gyro;

    // roughly 537.7, but ((((1+(46/17))) * (1+(46/11))) * 28) to be exact (on the site)
    // Link for motor:
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // Ticks Per Rotation (how many ticks in one full motor rotation)
    private static final double TPR = (1+((double)46/17)) * (1+((double)46/11)) * 28;// ticks per revolution
    // circumference of the pulley circle pulling the string in linear slides
    private static final double CIRCUMFERENCE = 112; // in mm
    // DON'T USE THIS, IF IT'S TOO MUCH IT MIGHT BREAK THE LINEAR SLIDE
    private int MAX_LINEAR_SLIDE_EXTENSION = 4240; // in ticks
    private int ZERO_TICKS_LINEAR_SLIDE = 10; // zero position of linear slides (not 0 due to overextension issues) - in ticks
    private static final double RADIUS = 4.8; // in cm
    private static final double PI=3.1415926535;
    private static final double WHEEL_CIRCUMFERENCE = 2*PI*RADIUS;
    BNO055IMU imu;
    Orientation lastAngles;
    double globalAngle;

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

        Odometry odo = new Odometry(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        // Start of OpMode
        while (opModeIsActive()) {

            odo.update();

            // move mecanum drivetrain using gamepad values
            if (gamepad1.left_trigger > .6f) {
                setPowerMecanumGamepad(.125);
            } else if (gamepad1.right_trigger > .6f) { // superspeed mode
                setPowerMecanumGamepad(.8);
            } else {
                setPowerMecanumGamepad(.5);
            }

            // controlling linear slides and intake -- gamepad 2

            //servo intake, servo outtake
            if (gamepad2.left_trigger > .6f) {
                setPowerServo(-.7);
            }
            else if (gamepad2.right_trigger > .6f) {
                setPowerServo(1);
            }
            else {// default to not having servo move (only move when triggered)
                setPowerServo(0);
            }

            // state variables (switching between them)
            // NO LOCKING
            // if (gamepad2.left_stick_button) {lock = !lock;}

            if (gamepad2.right_bumper) {
                continousMode = true;
            }
            if (gamepad2.left_bumper) {
                continousMode = false;
            }

            // do things depending on states
            position = LinearSlide.getCurrentPosition();
            telemetry.addLine(String.valueOf(position));
            telemetry.update();

            if (continousMode) {
                LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LinearSlide.setPower(-gamepad2.left_stick_y * .75);
            } else {

                //linear slides
                // height 1 (low junction)

                if (gamepad2.a) {
                    setSlideMMAbsolute(365);
                }

                // height 2 (medium junction)
                if (gamepad2.b) {
                    setSlideTicksAbsolute(3000);
                }

                // height 3 (high junction) -- 850mm, but we can't go that much yet
                if (gamepad2.y) {
                    setSlideMaxAbsolute();
                }

                // down from any position
                if (gamepad2.x) {
                    setSlideBottomAbsolute();
                }

                if (gamepad1.a){
                    turnPID(180);
                }
                if (gamepad1.b) {
                    turnPID(-90);
                }
                if (gamepad1.y) {
                    turnPID(1);
                }
                if(gamepad1.x) {
                    turnPID(90);
                }

            }

        }
    }

    //set functions
    public void setAndConfLinearSlide(DcMotor slide){
        LinearSlide = slide;
        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setPower(0);
    }

    public void setAndConfServos(CRServo left, CRServo right){
        leftServo = left;
        rightServo = right;
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


        imu = hardwareMap.get(BNO055IMU.class, "Gyro");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters); // init the gyro

        telemetry.addData("Calibrating imu: ", imu.getCalibrationStatus().toString());
        telemetry.addData("imu ready?: ", imu.isGyroCalibrated());

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
    public void setSlideTicksAbsolute(int ticks) {
        boolean down = ((ticks - LinearSlide.getCurrentPosition()) < 0)? true : false;
        PIDController slidePIDController;
        if (down) {
            slidePIDController = new PIDController(ticks, 0.0015, 0.0000, 0.2, false);
        }else{
            //slidePIDController = new PIDController(ticks, 0.004, 0.001, 0.2, false);
            slidePIDController = new PIDController(ticks, 0.008, 0.0000, 0.1, false);
        }
        telemetry.setMsTransmissionInterval(50);

        LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slidePower = 0;
        int currentTicks = LinearSlide.getCurrentPosition();

        while((Math.abs(currentTicks - ticks) > 10) && opModeIsActive()){
            telemetry.addLine("current: " + currentTicks);
            telemetry.addLine("setpoint" + ticks);
            telemetry.update();

            currentTicks = LinearSlide.getCurrentPosition();
            slidePower = slidePIDController.update(currentTicks);
            LinearSlide.setPower(slidePower);

            //teleop
            if (teleop) {
                if (gamepad1.left_trigger > .6f) {
                    setPowerMecanumGamepad(.125);
                } else if (gamepad1.right_trigger > .6f) { // superspeed mode
                    setPowerMecanumGamepad(.8);
                } else {
                    setPowerMecanumGamepad(.5);
                }
                //servo intake, servo outtake
                if (gamepad2.left_trigger > .6f) {setPowerServo(-1);}
                else if (gamepad2.right_trigger > .6f) {setPowerServo(.7);}
                else {setPowerServo(0);}

                if (gamepad1.a){
                    turnPID(180);
                }
                if (gamepad1.b) {
                    turnPID(-90);
                }
                if (gamepad1.y) {
                    turnPID(1);
                }
                if(gamepad1.x) {
                    turnPID(90);
                }
            }
        }

        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setTargetPosition(ticks);
        LinearSlide.setPower(slidePower);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    // set linear slide to certain tick position from MILLIMETER measurement upwards
    public void setSlideMMAbsolute(double mm)  {
        // convert from MM to ticks
        // first convert from mm to rotations (mm / CIRCUMFERENCE) = rotations
        // then convert from rotations to ticks ( rotations * TPR
        int ticksFromMM = (int)( (mm / CIRCUMFERENCE) * TPR);
        setSlideTicksAbsolute(ticksFromMM);
    }

    public void setSlideBottomAbsolute()  {
        setSlideTicksAbsolute(ZERO_TICKS_LINEAR_SLIDE);
        LinearSlide.setPower(0);
    }


    public void setSlideMaxAbsolute()  {
        setSlideTicksAbsolute(MAX_LINEAR_SLIDE_EXTENSION);
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

    public double getAngle(){
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void turnHeading(double angle, double pow) throws InterruptedException {
        double power = pow;
        double m_P = 5;
        double tol = 2.5;

        double err = (angle-this.getAngle());

        double prop = err/360;

        while(Math.abs(err)>tol){
            power = (m_P*prop);

            backRightMotor.setPower(-power);
            frontRightMotor.setPower(-power);

            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);

            err = (angle-this.getAngle());
            telemetry.addLine(String.valueOf(this.getAngle()));
            telemetry.update();
        }

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


    //public void setPowerLinearSlide(double power){
    //    LinearSlide.setPower(power);
    //}
    public double getAbsoluteAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }


    public void setMotorPower(double fLM, double bLM, double fRM, double bRM) {
        frontLeftMotor.setPower(fLM);
        backLeftMotor.setPower(bLM);
        frontRightMotor.setPower(fRM);
        backRightMotor.setPower(bRM);
    }

    // move servo for certain amount of (milliseconds) with (power)
    public void moveServo(long ms, double power) throws InterruptedException {
        setPowerServo(power);
        Thread.sleep(ms);
        setPowerServo(0);
    }
    public void turnPID(double degrees) {
        turnToPID(degrees);
    }

    void turnToPID(double targetAngle) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController pid = new PIDController(targetAngle, 0.023, 0.000, 0.006, true);
        //telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        //boolean mode = (teleop)? opModeIsActive() : true;
        //boolean mode = opModeIsActive();
        while ((Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) && opModeIsActive()) {
            //mode = (teleop)? opModeIsActive() : true;
            //mode = opModeIsActive();
            double motorPower = pid.update(getAbsoluteAngle());
            setMotorPower(motorPower, motorPower, -motorPower, -motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        setMotorPower(0, 0,0,0);
     }

}

