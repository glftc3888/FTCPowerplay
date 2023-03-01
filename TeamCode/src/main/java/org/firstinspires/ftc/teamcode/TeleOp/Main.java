package org.firstinspires.ftc.teamcode.TeleOp;

// Import modules
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.drive.opmode.LinearSlideHolder;

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

    private Lift lift = null;

    private DcMotor LinearSlide = null;
    private static Rev2mDistanceSensor distanceSensor = null;

    boolean teleop = false;

    // will use later... This is for angle (see code from 2last year)
    BNO055IMU Gyro;

    // roughly 537.7, but ((((1+(46/17))) * (1+(46/11))) * 28) to be exact (on the site)
    // Link for motor:
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // Ticks Per Rotation (how many ticks in one full motor rotation)
    private static final double TPR = (1+((double)46/17)) * (1+((double)46/11)) * 28;// ticks per revolution
    private static final double LinearSlideTPR = ((((1+((double)46/17))) * (1+((double)46/17))) * 28); //ticks per revolution according to gobilda site - 435 rpm
    // circumference of the pulley circle pulling the string in linear slides
    private static final double CIRCUMFERENCE = 112; // in mm
    // DON'T USE THIS, IF IT'S TOO MUCH IT MIGHT BREAK THE LINEAR SLIDE
    private int MAX_LINEAR_SLIDE_EXTENSION = (int) (4240 * (384.5 / 537.7)); // in ticks
    private int ZERO_TICKS_LINEAR_SLIDE = 10; // zero position of linear slides (not 0 due to overextension issues) - in ticks
    private static final double RADIUS = 4.8; // in cm
    private static final double PI=3.1415926535;
    private static final double WHEEL_CIRCUMFERENCE = 2*PI*RADIUS;
    BNO055IMU imu;
    Orientation lastAngles;
    double globalAngle;

    boolean continousMode = false;
    int position = 0;
    double ENDGAME = 90;

    //fancy distance sensors

    //controllers rumbling
    Gamepad.RumbleEffect rumbleSequence;


    @Override
    public void runOpMode() throws InterruptedException {


        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        teleop = true;

        //init the robot
        initRobot();

        // Wait for start
        waitForStart();
        runtime.reset();

        Odometry odo = new Odometry(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //NEW and HOT: Rumbling
        rumbleSequence = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 0, 250)
                .addStep(0, 1, 250)
                .addStep(1, 0, 250)
                .addStep(0, 1, 250)
                .build();

        lift.setZeroPosition();

        // Start of OpMode
        while (opModeIsActive()) {

            drive_centric();
            //field_centric(drive);
            //Distance Sensor Functions
            distanceIntakedRumble();

            /*
            // move mecanum drivetrain using gamepad values

            */

            // controlling linear slides and intake -- gamepad 2
            /*if ( Math.abs( runtime.seconds() - ENDGAME ) < 1){
                gamepad1.runRumbleEffect(rumbleSequence);
                gamepad2.runRumbleEffect(rumbleSequence);
            }

            if ( Math.abs( runtime.seconds() - (ENDGAME + 20) ) < 1){
                gamepad1.runRumbleEffect(rumbleSequence);
                gamepad2.runRumbleEffect(rumbleSequence);
            }*/

            //servo intake, servo outtake
            if (gamepad2.left_trigger > .6f) {intake(1);}
            else if (gamepad2.right_trigger > .6f) {outtake(.7);}
            else {kill_intake();}

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
            /*telemetry.addLine(String.valueOf(position));
            telemetry.update();*/

            if (continousMode) {
                //ADD: CONT MODE
                /*
                LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LinearSlide.setPower(-gamepad2.left_stick_y);
                 */
                lift.continousMode(gamepad2.left_stick_y);
            } else {
                //ADD: NON-CONT MODE
                //linear slides
                // height 1 (low junction)

                if (gamepad2.a){lift.setLowPosition();}
                if (gamepad2.b){lift.setMediumPosition();}
                if (gamepad2.y){lift.setHighPosition();}
                if (gamepad2.x){lift.setZeroPosition();}
                lift.update();
                /*
                if (gamepad2.a) {
                    setSlideTicksAbsolute( (int) ((ticksFromMM(375) * (384.5 / 537.7)) ));
                }

                // height 2 (medium junction)
                if (gamepad2.b) {
                    setSlideTicksAbsolute((int) (3000 * (384.5 / 537.7)));
                }

                // height 3 (high junction) -- 850mm, but we can't go that much yet
                if (gamepad2.y) {
                    setSlideTicksAbsolute(MAX_LINEAR_SLIDE_EXTENSION);
                }

                // down from any position
                if (gamepad2.x) {
                    setSlideTicksAbsolute(ZERO_TICKS_LINEAR_SLIDE);
                }*/

                //FLIP THIS HERE
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

    //DISTANCE SENSOR FUNCTIONS
    public boolean isIntaked(){
        return (distanceSensor.getDistance(DistanceUnit.MM) < 24);
    }

    public void distanceIntakedRumble(){
        if (isIntaked())
        {
            double INTAKE_RUMBLE = .1;

            gamepad1.setLedColor(1, .8, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, .8, 0, Gamepad.LED_DURATION_CONTINUOUS);

            gamepad1.rumble(INTAKE_RUMBLE, INTAKE_RUMBLE, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad2.rumble(INTAKE_RUMBLE, INTAKE_RUMBLE, Gamepad.RUMBLE_DURATION_CONTINUOUS);

        } else {
            gamepad1.setLedColor(.33, 0, .67, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(.33, 0, .67, Gamepad.LED_DURATION_CONTINUOUS);

            gamepad1.stopRumble();
            gamepad2.stopRumble();
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

        lift = new Lift();

        /*LinearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");
        // keep it reverse if you want positive ticks to move linear slide up
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        //when you're setting it up (in opMode)
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlide.setPower(0);*/

        //Use the linear slide from AUTO
        //LinearSlide = (new LinearSlideHolder()).getLinearSlide();

        leftServo = hardwareMap.crservo.get("left_servo");                 //left CR Servo
        rightServo = hardwareMap.crservo.get("right_servo");               //right CR Servo
        distanceSensor = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

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

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);


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
/*
    public void setSlideTicksAbsoluteEncoders(int ticks, double power){

        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setPower(power);
        LinearSlide.setTargetPosition(ticks);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(LinearSlide.isBusy()){
            //teleop
            if (teleop) {
                //drive centric not field centric
                drive_centric();

                //servo intake, servo outtake
                if (gamepad2.left_trigger > .6f) {intake(1);}
                else if (gamepad2.right_trigger > .6f) {setPowerServo(-.7);}
                else {setPowerServo(0);}

                distanceIntakedRumble();

                //BTW, THIS DOES NOT WORK ASYNCHRONOUSLY, IT WOULD STOP THE ENTIRE PID LOOP FOR TURNING
                //FLIP THIS HERE

                if (gamepad1.a){turnPID(180);}
                if (gamepad1.b) {turnPID(-90);}
                if (gamepad1.y) {turnPID(1);}
                if(gamepad1.x) {turnPID(90);}
                if(gamepad2.right_stick_button || gamepad2.left_stick_button){break;}


            }
        }
        LinearSlide.setPower(.1);
    }

    // Make sure the encoder cables are connected right, and the the forward/backward is in the right place
    // setSlideTicksAbsolute moves the linear slide to a certain tick POSITION (not BY a certain amount)
    public void setSlideTicksAbsolute(int ticks) {
        boolean down = ((ticks - LinearSlide.getCurrentPosition()) < 0)? true : false;
        boolean use_pid = down;
        PIDController slidePIDController;
        if (down) {
            slidePIDController = new PIDController(ticks, 0.0010, 0.0000, 0.02, false);
        }else{
            //slidePIDController = new PIDController(ticks, 0.004, 0.001, 0.2, false);
            slidePIDController = new PIDController(ticks, 0.009, 0.0000, .8, false);
        }

        if(!use_pid){
            setSlideTicksAbsoluteEncoders(ticks, 1);
            return;
        }

        LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slidePower = 0;
        int currentTicks = LinearSlide.getCurrentPosition();

        while((Math.abs(currentTicks - ticks) > 20) && opModeIsActive()){
            telemetry.addLine("current: " + currentTicks);
            telemetry.addLine("setpoint" + ticks);
            telemetry.update();

            //teleop
            if (teleop) {
                //drive centric not field centric
                drive_centric();
                //servo intake, servo outtake
                if (gamepad2.left_trigger > .6f) {intake(1);}
                else if (gamepad2.right_trigger > .6f) {setPowerServo(-.7);}
                else {setPowerServo(0);}

                distanceIntakedRumble();

                //BTW, THIS DOES NOT WORK ASYNCHRONOUSLY, IT WOULD STOP THE ENTIRE PID LOOP FOR TURNING
                //FLIP THIS HERE
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

                if(gamepad2.right_stick_button || gamepad2.left_stick_button){
                    break;
                }

            }

            currentTicks = LinearSlide.getCurrentPosition();
            slidePower = slidePIDController.update(currentTicks);
            LinearSlide.setPower(slidePower);
        }

        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setTargetPosition(ticks);
        LinearSlide.setPower(slidePower);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }*/

    public void intake(double power){
        setPowerServo(power);
    }

    public void outtake(double power){
        setPowerServo(-power);
    }

    public void kill_intake(){
        setPowerServo(0);
    }

    public int ticksFromMM(double mm){
        return (int)( (mm / CIRCUMFERENCE) * LinearSlideTPR);
    }
/*
    // set linear slide to certain tick position from MILLIMETER measurement upwards
    public void setSlideMMAbsolute(double mm)  {
        // convert from MM to ticks
        // first convert from mm to rotations (mm / CIRCUMFERENCE) = rotations
        // then convert from rotations to ticks ( rotations * TPR
        setSlideTicksAbsolute(ticksFromMM(mm));
    }

    public void setSlideBottomAbsolute()  {
        setSlideTicksAbsolute(ZERO_TICKS_LINEAR_SLIDE);
        LinearSlide.setPower(0);
    }


    public void setSlideMaxAbsolute()  {
        setSlideTicksAbsolute(MAX_LINEAR_SLIDE_EXTENSION);
    }

*/

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
            /*telemetry.addLine(String.valueOf(this.getAngle()));
            telemetry.update();*/
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
    public void moveServo(long ms, double power)  {
        setPowerServo(power);
        sleep(ms);
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

            /*telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();*/
        }
        setMotorPower(0, 0,0,0);
     }
    public void field_centric(SampleMecanumDrive drive){

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );


        drive.update();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

    public void drive_centric(){
        if (gamepad1.left_trigger > .6f) {
            setPowerMecanumGamepad(.15);
        }
        else if (gamepad1.right_trigger > .6f) { // superspeed mode
            setPowerMecanumGamepad(1);
        }
        else {
            setPowerMecanumGamepad(.8);
        }
    }

    // LIFT CLASS FOR ASYNCHRONOUS

    // LIFT CLASS FOR STATE MACHINE
    public class Lift {
        PIDController slidePIDController;
        public int targetPos = 0;
        public boolean isStabilized = false;
        public double currentCone = 5;

        private static final double LinearSlideTPR = ((((1+((double)46/17))) * (1+((double)46/17))) * 28); //ticks per revolution according to gobilda site - 435 rpm
        // circumference of the pulley circle pulling the string in linear slides
        private static final double CIRCUMFERENCE = 112; // in mm

        public DcMotor LinearSlide;
        private int LOW_EXTENSION =  (int)(385 / CIRCUMFERENCE * LinearSlideTPR);
        private int HIGH_EXTENSION = (int) (4240 * (384.5 / 537.7)); // in ticks
        private int MEDIUM_EXTENSION = (int) (3000 * (384.5 / 537.7)); // in ticks
        private int MAX_STACK_VAL = (int) ((5.6 * 2.54 * 10.0 - 30) / CIRCUMFERENCE * LinearSlideTPR);
        private int HEIGHT_CONE_TICKS = 505;
        //BTW THIS IS WRONG, it takes high extension for some reason?
        private double HEIGHT_FROM_GRAB_TO_TOP_TICKS = HEIGHT_CONE_TICKS - (int) (4240 * (384.5 / 537.7) * 1/5);

        public Lift(){
            isStabilized = false;
            LinearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
            setAndConfLinearSlide(LinearSlide);
        }

        public DcMotor getLinearSlide(){
            return LinearSlide;
        }

        public boolean isBusy(){
            return !isStabilized;
        }

        public double getAdditiveFactorY() {
            // 5th -> 0, 4th -> 1, etc...
            return .37 * (5 - currentCone); // in inches
        }

        public double getAdditiveFactorX() {
            // 5th -> 0, 4th -> 1, etc...
            return .3 * (5 - currentCone); // in inches
        }

        public void continousMode(double input){
            LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LinearSlide.setPower(-input);
        }

        public void setTargetPosition(int ticks){
            targetPos = ticks;
            isStabilized = false;
            LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boolean down = ((ticks - LinearSlide.getCurrentPosition()) < 0)? true : false;
            if (down) {
                slidePIDController = new PIDController(targetPos, 0.0015, 0.0000, 0.2, false);
            }else{
                //slidePIDController = new PIDController(ticks, 0.004, 0.001, 0.2, false);
                slidePIDController = new PIDController(targetPos, 0.008, 0.0000, 0.1, false);
            }
        }

        public void setLowPosition(){
            setTargetPosition(LOW_EXTENSION);
        }
        public void setMediumPosition(){
            setTargetPosition(MEDIUM_EXTENSION);
        }
        public void setHighPosition(){
            setTargetPosition(HIGH_EXTENSION);
        }

        public boolean setStackPosition(){
            double l_ratio = currentCone / 5;
            telemetry.addLine(String.valueOf(currentCone));
            telemetry.update();

            if (currentCone == 2){
                l_ratio = 1.7/5;
            }else if(currentCone == 1){
                l_ratio = 0.02/5;
            }
            setTargetPosition((int) (MAX_STACK_VAL * l_ratio));

            currentCone--;
            return !(currentCone >= 0);
        }

        public void setZeroPosition(){
            setTargetPosition(0);
        }

        public boolean hasSurpassedStack(){
            return (LinearSlide.getCurrentPosition() > ( HEIGHT_FROM_GRAB_TO_TOP_TICKS + (currentCone / 5) ) );
        }

        public void update() {
            int currentTicks = LinearSlide.getCurrentPosition();
            double slidePower = slidePIDController.update(currentTicks);
            int TOLERANCE = 20;
            if (!isStabilized) {
                if ((Math.abs(currentTicks - targetPos) > TOLERANCE)) {
                    LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LinearSlide.setPower(slidePower);
                }
                // two "if"s intead of if -> else because we need to do one right after the other
                if (! (Math.abs(currentTicks - targetPos) > TOLERANCE) ) {
                    LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LinearSlide.setTargetPosition(targetPos);
                    LinearSlide.setPower(.1);
                    LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    isStabilized = true;
                }
            }

        }
    }

}

