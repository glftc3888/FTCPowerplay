package org.firstinspires.ftc.teamcode.Autonomous.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


// This is an example of a more complex path to really test the tuning.
@Autonomous(group = "drive")
public class AutoLeftSpline extends Main {

    enum State {
        IDLE,

        //FOR ALL PRELOAD
        PRELOAD_TRAJ_1,
        PRELOAD_TRAJ_2,
        PRELOAD_TRAJ_3,

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
                    lift.emergency();
                    break;
                case NOTHING: // default, keep at -1
                    parkY = -1;
                    break;
            }
        }


        telemetry.addLine("READY!!!!!: " + parkY);
        telemetry.addLine("EMERGENCY: " + lift.lessCones);
        telemetry.update();

        // BEGINNING IS P2.end() -- add some wiggle
        Trajectory preload_trajectory_p3 = drive.trajectoryBuilder(new Pose2d(-9.83 + .05, -14.93 - .03, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-24, -11.20 + .5))
                .build();

        //BEGINNING IS P1.end() -- add some wiggle
        Trajectory preload_trajectory_p2 = drive.trajectoryBuilder(new Pose2d(-11.94 + .03, -58.83 - .03, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-9.83, -14.93))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(preload_trajectory_p3))
                .build();

        //BEGINS WHERE IT BEGINS
        Trajectory preload_trajectory_p1 = drive.trajectoryBuilder(new Pose2d(-36.00, -63.37, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-11.94, -58.83))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(preload_trajectory_p2))
                .build();


        TrajectorySequence preload_trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.37, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-11.94, -58.83))
                .lineToConstantHeading(new Vector2d(-9.83, -14.93))
                .lineToConstantHeading(new Vector2d(-23.3, -11.20 + .5))
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 53;}})
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 39;
                    }
                })
                .build();

        /*TrajectorySequence preload_trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.37, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-11.94, -56.37, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(-9.83, -13.87, Math.toRadians(90.00)), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(-23.5, -11.20 + .5))
                .build();*/

        // change to start going to stack after end of PRELOAD_TRAJECTORY_P1
        TrajectorySequence go_stack_preload_trajectory = drive.trajectorySequenceBuilder(preload_trajectory_p3.end())
                .lineToSplineHeading(new Pose2d(-36.18, -14.58, Math.toRadians(135.00)))
                //.lineToLinearHeading(new Pose2d(-63.97, -12.25, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-64.05, -12.25 - 1, Math.toRadians(180.00)))
                .build();

// previous one
        /*
        TrajectorySequence go_low_junction_trajectory = drive.trajectorySequenceBuilder(go_stack_preload_trajectory.end())
                .lineToConstantHeading(new Vector2d(-64 + 4, -12.74))
                .lineToLinearHeading(new Pose2d(-47, -14.65 - 1.3, Math.toRadians(270.00)))
                .build();

         */

        TrajectorySequence go_low_junction_trajectory = drive.trajectorySequenceBuilder(new Pose2d(-64.00, -12.25, Math.toRadians(180.00)))
                //.lineToConstantHeading(new Vector2d(-64 + 4, -12.23))
                .splineToLinearHeading(new Pose2d(-64 + 4, -12.23, Math.toRadians(180)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-48.03 + .07, -12.25, Math.toRadians(270.00)))
                .build();


        TrajectorySequence go_stack_low_trajectory = drive.trajectorySequenceBuilder(go_low_junction_trajectory.end())
                .lineTo(new Vector2d(-47.88, -12.16))
                //.lineToSplineHeading(new Pose2d(-63.98, -12.22, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-63.54, -12.22, Math.toRadians(180.00)))
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


        currentState = State.PRELOAD_TRAJ_1;
        //currentState = State.PRELOAD_TRAJ;

        //drive.setPoseEstimate(preload_trajectory.start());
        //drive.followTrajectorySequenceAsync(preload_trajectory);
        drive.setPoseEstimate(preload_trajectory_p1.start());
        drive.followTrajectoryAsync(preload_trajectory_p1);

        lift.setHighPosition();

        while(opModeIsActive() && !isStopRequested()) {

            telemetry.addLine(currentState.name());
            telemetry.update();

            switch (currentState) {
                case PRELOAD_TRAJ_1:
                    if(!drive.isBusy() && !lift.isBusy()){
                        currentState = State.OUTTAKE_PRELOAD;
                    }
                    break;

                case OUTTAKE_PRELOAD:
                    telemetry.addLine("IN OUTTAKE");
                    telemetry.update();
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
                    while(runtime.milliseconds() < 300){
                        lift.update();
                    }
                    setPowerServo(0);

                    boolean finished_with_cycle = lift.setStackPosition();

                    //CHANGE TRAJECTORY TO ADD - COMPENSATING FOR DRIFT
                    double ADDITIVE_FACTOR_Y = lift.getAdditiveFactorY();
                    double ADDITIVE_FACTOR_X = lift.getAdditiveFactorX();

                    //logic to finish cycling all 5 cones
                    if (finished_with_cycle) {
                        currentState = State.PARK;

                        // ADD additive factor
                        park_trajectory = drive.trajectorySequenceBuilder(go_low_junction_trajectory.end())
                                //ADD added 3 more inches to all
                                .lineToConstantHeading(new Vector2d(-12 + parkY + 3, -12 - 1))
                                //.lineToConstantHeading(new Vector2d(-12 + parkY - ADDITIVE_FACTOR_X, -12 - ADDITIVE_FACTOR_Y))
                                .setVelConstraint(new TrajectoryVelocityConstraint() {
                                    @Override
                                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                        return 50;
                                    }
                                })
                                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                                    @Override
                                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                        return 45;
                                    }
                                })
                                .build();

                        drive.followTrajectorySequenceAsync(park_trajectory);
                        //lift.setStackPosition();
                        lift.setZeroPosition();
                    }else{
                        currentState = State.GO_STACK_TRAJ;

                        // ADD additive factor
                        go_stack_low_trajectory = drive.trajectorySequenceBuilder(go_low_junction_trajectory.end())
                                .lineTo(new Vector2d(-47.88, -12.16))
                                .lineToSplineHeading(new Pose2d(-64.05 - ADDITIVE_FACTOR_X, -12.23 - ADDITIVE_FACTOR_Y, Math.toRadians(180.00)))
                                //.lineToSplineHeading(new Pose2d(-63.98 - ADDITIVE_FACTOR_X, -12.22 - ADDITIVE_FACTOR_Y, Math.toRadians(180.00)))
                                .build();

                        drive.followTrajectorySequenceAsync(go_stack_low_trajectory);
                    }
                    break;
                case GO_STACK_TRAJ:
                    if (!drive.isBusy() && !lift.isBusy()){
                        //remove intake (DON'T DO IT) -- see what happens
                        //moveServo(100, .8);
                        currentState = State.UNSTACK_WHILE_MOVING_UP;
                        lift.setLowPosition();
                    }
                    break;
                case UNSTACK_WHILE_MOVING_UP:
                    //CHANGE TRAJECTORY TO ADD - COMPENSATING FOR DRIFT
                    ADDITIVE_FACTOR_Y = lift.getAdditiveFactorY();
                    ADDITIVE_FACTOR_X = lift.getAdditiveFactorX();

                    go_low_junction_trajectory = drive.trajectorySequenceBuilder(new Pose2d(-64.00, -12.25, Math.toRadians(180.00)))
                            //.lineToConstantHeading(new Vector2d(-64 + 4, -12.23))
                            .splineToLinearHeading(new Pose2d(-64 + 4, -12.23, Math.toRadians(180)), Math.toRadians(180))
                            //.lineToLinearHeading(new Pose2d(-48.03 - ADDITIVE_FACTOR_X, -12.25 - 1.95 - ADDITIVE_FACTOR_Y, Math.toRadians(270.00)))
                            .lineToLinearHeading(new Pose2d(-48.03 - ADDITIVE_FACTOR_X, -12.25 - 1.0 - ADDITIVE_FACTOR_Y, Math.toRadians(270.00)))

                            .build();

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

        public int lessCones = 0;

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

        public void emergency(){
            lessCones = 1;
        }

        public DcMotor getLinearSlide(){
            return LinearSlide;
        }

        public boolean isBusy(){
            return !isStabilized;
        }

        public double getAdditiveFactorY() {
            // 5th -> 0, 4th -> 1, etc...
            return .4 * (5 - currentCone); // in inches
        }

        public double getAdditiveFactorX() {
            // 5th -> 0, 4th -> 1, etc...
            return .08 * (5 - currentCone); //.09 * (5 - currentCone); // in inches
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

        public void setZeroPosition(){
            setTargetPosition(10);
        }


        public boolean setStackPosition(){
            double l_ratio = currentCone / 5;

            if (currentCone == 2){
                l_ratio = 1.7/5;
            }else if(currentCone == 1){
                l_ratio = 0.02/5;
            }
            setTargetPosition((int) (MAX_STACK_VAL * l_ratio));

            currentCone--;
            return !(currentCone >= 1 + lessCones);
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

