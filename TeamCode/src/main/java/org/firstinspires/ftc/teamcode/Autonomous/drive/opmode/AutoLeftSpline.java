package org.firstinspires.ftc.teamcode.Autonomous.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.SleeveDetection;
import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Main;
import org.firstinspires.ftc.teamcode.TeleOp.PIDController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


// This is an example of a more complex path to really test the tuning.
@Autonomous(group = "drive")
public class AutoLeftSpline extends Main {

    enum State {
        IDLE,

        //Trajectories
        PRELOAD_TRAJ,
        GO_STACK_TRAJ,
        GO_HIGH_TRAJ,
        OUTTAKE_PRELOAD,
        PARK,

        //Stack
        UNSTACK_WHILE_MOVING_UP,

        //Intake/Slide
        INTAKE,
        OUTTAKE,
    }

    // Declaration of global variables
    ElapsedTime runtime = new ElapsedTime();

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    //INITIALIZATION

    LinearSlideHolder slideHolder = new LinearSlideHolder();

    // camera setup
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    //drive setup
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {


        // CAMERA
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //INITIALIZE
        //Initialize the drive, lift, and servo
        drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift();
        Servos servo = new Servos();

        // no need for waitForStart() with while loop
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("YEL: ", sleeveDetection.yelPercent());
            telemetry.addData("CYA: ", sleeveDetection.cyaPercent());
            telemetry.addData("MAG: ", sleeveDetection.magPercent());
            telemetry.update();
        }
        if (isStopRequested()) return;

        // get parking position enum
        SleeveDetection.ParkingPosition parkingPosition;
        // camera code
        double parkY = -1;
        while (parkY == -1) { //while
            parkingPosition = sleeveDetection.getPosition();
            switch (parkingPosition) {
                case LEFT:
                    parkY = -23.5 * 2;
                    break;
                case CENTER:
                    parkY = -23.5;
                    break;
                case RIGHT:
                    parkY = 0;
                    break;
                case NOTHING: // default, keep at -1
                    parkY = -1;
                    break;
            }
        }

        telemetry.addLine("READY!!!!! " + parkY);
        telemetry.update();

        TrajectorySequence preload_trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.37, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-11.94, -58.83))
                .lineToConstantHeading(new Vector2d(-9.83, -14.93))
                .lineToConstantHeading(new Vector2d(-23.33, -11.20 + .2))
                .build();

        TrajectorySequence go_stack_preload_trajectory = drive.trajectorySequenceBuilder(preload_trajectory.end())
                .lineToSplineHeading(new Pose2d(-36.18, -14.58, Math.toRadians(135.00)))
                .lineToLinearHeading(new Pose2d(-63.30, -12.74, Math.toRadians(180.00)))
                .build();

        TrajectorySequence go_low_junction_trajectory = drive.trajectorySequenceBuilder(go_stack_preload_trajectory.end())
                .lineToLinearHeading(new Pose2d(-48.47, -14.75 - .1, Math.toRadians(270.00)))
                .build();

        TrajectorySequence go_stack_low_trajectory = drive.trajectorySequenceBuilder(go_low_junction_trajectory.end())
                .lineTo(new Vector2d(-47.68, -12.16))
                .lineToSplineHeading(new Pose2d(-63.30, -12.74, Math.toRadians(180.00)))
                .build();

        /*

        TrajectorySequence go_stack_trajectory = drive.trajectorySequenceBuilder(preload_trajectory.end())
                // GO TO STACK
                .lineToSplineHeading(new Pose2d(-38.81, -12.12, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-62.87, -11.77, Math.toRadians(180.00)))
                .build();

        TrajectorySequence go_high_junction_trajectory = drive.trajectorySequenceBuilder(go_stack_trajectory.end())
                // GOING TO SCORE HIGH
                .lineTo(new Vector2d(-41.62, -12.64))
                .lineToSplineHeading(new Pose2d(-24.06, -9.83, Math.toRadians(90.00)))
                .build();
        */

        TrajectorySequence park_trajectory = drive.trajectorySequenceBuilder(go_low_junction_trajectory.end())
                .lineToConstantHeading(new Vector2d(-12 + parkY, -12))
                .build();

        drive.setPoseEstimate(preload_trajectory.start());

        /*IDLE,

        //Trajectories
        PRELOAD_TRAJ,
        GO_STACK_TRAJ,
        GO_HIGH_TRAJ,
        PARK,
        OUTTAKE_PRELOAD,

        //Stack
        UNSTACK_WHILE_MOVING_UP,

        //Intake/Slide
        INTAKE,
        OUTTAKE,*/

        currentState = State.PRELOAD_TRAJ;
        drive.followTrajectorySequenceAsync(preload_trajectory);
        lift.setHighPosition();

        while(opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case PRELOAD_TRAJ:
                    telemetry.addLine(String.valueOf(lift.isBusy()));
                    telemetry.addLine("Set Ticks:" + (int) (4240 * (384.5 / 537.7)));
                    telemetry.addLine("Actual Ticks:" + lift.LinearSlide.getCurrentPosition());
                    telemetry.update();

                    if (!drive.isBusy() && !lift.isBusy()){
                        currentState = State.OUTTAKE_PRELOAD;
                    }
                    break;
                case OUTTAKE_PRELOAD:
                    setPowerServo(-1);
                    runtime.reset();
                    while(runtime.milliseconds() < 1000){
                        lift.update();
                    }
                    setPowerServo(0);
                    //moveServo(1300, -1);
                    lift.setStackPosition();
                    currentState = State.GO_STACK_TRAJ;
                    drive.followTrajectorySequenceAsync(go_stack_preload_trajectory);
                    break;
                case OUTTAKE:
                    setPowerServo(-1);
                    runtime.reset();
                    while(runtime.milliseconds() < 1000){
                        lift.update();
                    }
                    setPowerServo(0);

                    boolean finished_with_cycle = lift.setStackPosition();
                    //logic to finish cycling all 5 cones
                    if (finished_with_cycle) {
                        currentState = State.PARK;
                        drive.followTrajectorySequenceAsync(park_trajectory);
                        lift.setStackPosition();
                    }else{
                        currentState = State.GO_STACK_TRAJ;
                        drive.followTrajectorySequenceAsync(go_stack_low_trajectory);
                    }
                    break;
                case GO_STACK_TRAJ:
                    if (!drive.isBusy() && !lift.isBusy()){
                        moveServo(150, 1);
                        currentState = State.UNSTACK_WHILE_MOVING_UP;
                        lift.setLowPosition();
                    }
                    break;
                case UNSTACK_WHILE_MOVING_UP:
                    if (lift.hasSurpassedStack()){
                        currentState = State.GO_HIGH_TRAJ;
                        drive.followTrajectorySequenceAsync(go_low_junction_trajectory);
                    }
                case GO_HIGH_TRAJ:
                    if (!drive.isBusy() && !lift.isBusy()){
                        currentState = State.OUTTAKE;
                    }
                    break;
                case PARK:
                    if (!drive.isBusy() && !lift.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    slideHolder.setLinearSlide(lift.getLinearSlide());
                    break;
            }

            drive.update();
            lift.update();
        }

    }

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
        private int LOW_EXTENSION =  (int)(375 / CIRCUMFERENCE * LinearSlideTPR);
        private int HIGH_EXTENSION = (int) (4240 * (384.5 / 537.7)); // in ticks
        private int MAX_STACK_VAL = (int) ((5.6 * 2.54 * 10.0 - 30) / CIRCUMFERENCE * LinearSlideTPR);
        private int HEIGHT_CONE_TICKS = 505;
        private double HEIGHT_FROM_GRAB_TO_TOP_TICKS = HEIGHT_CONE_TICKS - (int) (4240 * (384.5 / 537.7) * 1/5);

        public Lift(){
            LinearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
            setAndConfLinearSlide(LinearSlide);
        }

        public DcMotor getLinearSlide(){
            return LinearSlide;
        }

        public boolean isBusy(){
            return !isStabilized;
        }

        public void setTargetPosition(int ticks){
            targetPos = ticks;
            boolean down = ((ticks - LinearSlide.getCurrentPosition()) < 0)? true : false;
            if (down) {
                slidePIDController = new PIDController(targetPos, 0.0015, 0.0000, 0.2, false);
            }else{
                //slidePIDController = new PIDController(ticks, 0.004, 0.001, 0.2, false);
                slidePIDController = new PIDController(targetPos, 0.008, 0.0000, 0.1, false);
            }
            LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setLowPosition(){
            setTargetPosition(LOW_EXTENSION);
        }
        public void setHighPosition(){
            setTargetPosition(HIGH_EXTENSION);
        }

        public boolean setStackPosition(){
            double l_ratio = currentCone / 5;
            if (currentCone <= 2){
                l_ratio = 1/5;
            }else if(currentCone == 1){
                l_ratio = 0.03;
            }
            setTargetPosition((int) (MAX_STACK_VAL * l_ratio));

            currentCone--;
            return !(currentCone >= 0);
        }

        public boolean hasSurpassedStack(){
            return (LinearSlide.getCurrentPosition() > ( HEIGHT_FROM_GRAB_TO_TOP_TICKS + (currentCone / 5) ) );
        }

        public void update() {
            int currentTicks = LinearSlide.getCurrentPosition();
            double slidePower = slidePIDController.update(currentTicks);
            if ((Math.abs(currentTicks - targetPos) > 20)){
                LinearSlide.setPower(slidePower);
            }else if (!isStabilized){
                LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LinearSlide.setTargetPosition(targetPos);
                LinearSlide.setPower(.2);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                isStabilized = true;
            }

        }
    }
    // INTAKE CLASS FOR STATE MACHINE
    public class Servos {

        public CRServo leftServo, rightServo;

        public Servos(){
            leftServo = hardwareMap.crservo.get("left_servo");
            rightServo = hardwareMap.crservo.get("right_servo");
            setAndConfServos(leftServo, rightServo);
        }

        public void intake(int ms) throws InterruptedException {
            //right trigger
            setPowerServo(-1);
            //Thread.sleep(ms);
            //kill();
        }

        public void outtake(int ms) throws InterruptedException {
            //right trigger
            setPowerServo(1);
            //Thread.sleep(ms);
            //kill();
        }

        public void kill(){
            setPowerServo(0);
        }
    }
}

