package org.firstinspires.ftc.teamcode.Autonomous.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.SleeveDetection;
import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Main;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Queue;


// This is an example of a more complex path to really test the tuning.
@Autonomous(group = "drive")
public class AutoLeftSpline extends Main {

    //only servo and linear slide initialization
    private static CRServo leftServo = null;
    private static CRServo rightServo = null;
    private DcMotor LinearSlide = null;
    public static double MAX_STACK_VAL = 5.6 * 2.54 * 10.0 - 30;

    // camera setup
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    //drive setup
    SampleMecanumDrive drive;

    public void initRobotCool(){
        setAndConfLinearSlide(hardwareMap.get(DcMotor.class, "linear_slide"));
        setAndConfServos(hardwareMap.crservo.get("left_servo"), hardwareMap.crservo.get("right_servo"));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        if (isStopRequested()) return;

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

        initRobotCool();

        // no need for waitForStart() with while loop
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("YEL: ", sleeveDetection.yelPercent());
            telemetry.addData("CYA: ", sleeveDetection.cyaPercent());
            telemetry.addData("MAG: ", sleeveDetection.magPercent());
            telemetry.update();
        }

        // get parking position enum
        SleeveDetection.ParkingPosition parkingPosition;

        // camera code
        double parkY = -1;
        while (parkY == -1) { //while
            parkingPosition = sleeveDetection.getPosition();
            switch (parkingPosition) {
                case LEFT:
                    parkY = 23.5 + 4;
                    break;
                case CENTER:
                    parkY = 0;
                    break;
                case RIGHT:
                    parkY = -23.5 - 4;
                    break;
                case NOTHING: // default, keep at -1
                    parkY = -1;
                    break;
            }
        }

        telemetry.addLine("READY!!!!!" + parkY);
        telemetry.update();

        //multiple trajectories

        TrajectorySequence preloadTraj = drive.trajectorySequenceBuilder(new Pose2d())

                // push the cone away
                .lineToConstantHeading(new Vector2d(23.5 * 2.5 - 5, 0))

                // move slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideTicksAbsolute(3000);
                })
                // move and turn to align to first pole
                .lineToLinearHeading(new Pose2d(23.5 * 2 - 9, (23.5 * .5 - 10) * -1, Math.toRadians(90 * -1)))
                //outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(1);
                }).waitSeconds(1)
                .build();


        /*TrajectorySequence preloadTraj = drive.trajectorySequenceBuilder(new Pose2d())
                // push the cone away
                .lineTo(new Vector2d(23.5 * 2.5 - 5, 0))
                // move slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideTicksAbsolute(3000);
                })
                // move and turn to align to first pole
                .lineToLinearHeading(new Pose2d(23.5 * 2 - 6.5, (23.5 * .5 - 11.5) * -1, Math.toRadians(90 * -1)))
                //outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(1);
                }).waitSeconds(1)
                .build();*/

        TrajectorySequence positioningTraj = drive.trajectorySequenceBuilder(preloadTraj.end())
                //Move to correct right height for CONE 5
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(MAX_STACK_VAL);
                })

                // Move back to not bump into the pole
                .lineTo(new Vector2d(23.5 * 2 - 4.5, (23.5 * .5 - 12) * -1))

                // Position yourself to be at center - START OF CYCLE
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 10, 0 * -1, Math.toRadians(-90 * -1)))
                .build();

        /*
        TrajectorySequence cycleTraj5 = buildCycleTraj(4.0/5, positioningTraj.end());
        TrajectorySequence cycleTraj4 = buildCycleTraj(3.0/5, cycleTraj5.end());
        TrajectorySequence cycleTraj3 = buildCycleTraj(2.0/5, cycleTraj4.end());
         */

        TrajectorySequence cycleTrajHigh = buildHighCycleTraj(4.0/5, positioningTraj.end());
        TrajectorySequence cycleTrajLow = buildLowCycleTraj(0.0, cycleTrajHigh.end());


        TrajectorySequence parkingTraj = drive.trajectorySequenceBuilder(cycleTrajLow.end())

                //beautiful parking
                .lineToConstantHeading(new Vector2d(23.5 * 2.5 - 5, parkY)).setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 53;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 45;
                    }
                })
                //move linear slide down - DON'T NEED THIS ANYMORE (SLIDE GOES DOES AFTER TRAJECTORY LOW)
                /*.UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideBottomAbsolute();
                }).waitSeconds(.2)
                */

                .build();



        while(opModeIsActive() && !isStopRequested()) {


            drive.followTrajectorySequence(preloadTraj);
            drive.followTrajectorySequence(positioningTraj);
            drive.followTrajectorySequence(cycleTrajHigh);
            drive.followTrajectorySequence(cycleTrajLow);
            drive.followTrajectorySequence(parkingTraj);


            break;
        }

    }

    public TrajectorySequence buildHighCycleTraj(double constant, Pose2d end) {
        TrajectorySequence ctraj = drive.trajectorySequenceBuilder(end)
                // Go to cone stack
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 6.75, (-23.5 - 3.5) * -1, Math.toRadians(90)))
                // intake for a bit
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(-.3);
                }).waitSeconds(.3)

                // stop and move linear slide up (TO CORRECT ONE TO SAVE TIME)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMaxAbsolute();
                }).waitSeconds(.2)

                // move an inch back in order to not bump into field barrier when going up
                .lineTo(new Vector2d(23.5 * 2.5 - 6.75, (-23.5 - 1.25) * -1))

                // THIS CHANGES BUT SHOULD BE EASY MONEY
                // Go to (A LITTLE BIT BACK OF) junction (and turn to face it)
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 8, (23.5*.5 + 1.5) * -1, Math.toRadians(0)))
                // Go to (EXACTLY AT) junction
                .lineToConstantHeading(new Vector2d(23.5 * 2.5 - 8 + 4.2, (23.5*.5 + 1.5) * -1))
                //outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //HYPOTHESIS: Full power = more distance?
                    setPowerServo(1);
                }).waitSeconds(.6)
                //stop outtake & move linear slide to prepare for next cycle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(MAX_STACK_VAL * constant);
                }).waitSeconds(.2)
                // GO to (A LITTLE BIT BACK OF/AGAIN) junction
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 8, (23.5*.5 + 3.25) * -1, Math.toRadians(0)))
                .build();

        return ctraj;
    }


    public TrajectorySequence buildLowCycleTraj(double constant, Pose2d end){

        TrajectorySequence ctraj = drive.trajectorySequenceBuilder(end)
                // Go to cone stack
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 6.75, (-23.5 - 3.5) * -1, Math.toRadians(90)))
                // intake for a bit
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(-.3);
                }).waitSeconds(.3)
                // stop and move linear slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(350);
                }).waitSeconds(.2)
                // move an inch back in order to not bump into field barrier when going up
                .lineTo(new Vector2d(23.5 * 2.5 - 6.75, (-23.5 - 1.25) * -1))
                // Drop at the junction (and turn to face it)
                .lineToSplineHeading(new Pose2d(23.5 * 2.5 - 8 + .5, (-23.5*.5 - .25) * -1, Math.toRadians(180)))
                //outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //HYPOTHESIS: Full power = more distance?
                    setPowerServo(1);
                }).waitSeconds(.6)
                //stop outtake & move linear slide to prepare for next cycle
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(MAX_STACK_VAL * constant);
                }).waitSeconds(.2)
                .build();

        return ctraj;
    }
}

