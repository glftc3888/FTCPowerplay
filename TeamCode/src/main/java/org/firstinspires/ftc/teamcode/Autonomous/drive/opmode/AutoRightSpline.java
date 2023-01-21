package org.firstinspires.ftc.teamcode.Autonomous.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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


// This is an example of a more complex path to really test the tuning.
@Autonomous(group = "drive")
public class AutoRightSpline extends Main {

    //only servo and linear slide initialization
    private static CRServo leftServo = null;
    private static CRServo rightServo = null;
    private DcMotor LinearSlide = null;

    // camera setup
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    public void initRobotCool(){
        setAndConfLinearSlide(hardwareMap.get(DcMotor.class, "linear_slide"));
        setAndConfServos(hardwareMap.crservo.get("left_servo"), hardwareMap.crservo.get("right_servo"));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initRobotCool();

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

        // get parking position enum
        SleeveDetection.ParkingPosition parkingPosition;

        // camera code
        double parkY = -1;
        while (parkY == -1) {
            parkingPosition = sleeveDetection.getPosition();
            switch (parkingPosition) {
                case LEFT:
                    parkY = 23.5;
                    break;
                case CENTER:
                    parkY = 0;
                    break;
                case RIGHT:
                    parkY = -23.5;
                    break;
                case NOTHING: // default, keep at -1
                    parkY = -1;
                    break;
            }
        }


        // no need for waitForStart() with while loop
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("YEL: ", sleeveDetection.yelPercent());
            telemetry.addData("CYA: ", sleeveDetection.cyaPercent());
            telemetry.addData("MAG: ", sleeveDetection.magPercent());
            telemetry.update();
        }

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                // go to middle of next tile
                .lineToConstantHeading(new Vector2d(23.5 * 1, 0))
                // move slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideTicksAbsolute(3000);
                })
                // move to align to pole
                .lineToLinearHeading(new Pose2d(23.5 * 2 - 8 + 2, 23.5 * .5 - 10, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(1);
                }).waitSeconds(1)
                //right height for CONE 5
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(5.6 * 2.54 * 10.0);
                })
                // go back a bit
                .lineTo(new Vector2d(23.5 * 2 - 8 + 2, 23.5 * .5 - 12))

                // now position yourself - START OF CYCLE
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 10, 0, Math.toRadians(-90)))
                // then go forward
                .lineTo(new Vector2d(23.5 * 2.5 - 8, -23.5 - 3))
                // intake for a bit
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(-.3);
                }).waitSeconds(.2)
                // move an inch back in order to not bump into field barrier when going up
                .lineTo(new Vector2d(23.5 * 2.5 - 8, -23.5 - 1.5))
                // stop and move linear slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    //CHANGE: make it -10 mm down to intake
                    setSlideMMAbsolute(355);
                }).waitSeconds(.2)
                // go to the junction to drop
                // CHANGE: A little more forward
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 11.5, -23.5*.5, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(.5);
                }).waitSeconds(1.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                }).waitSeconds(.2)
                //go back to center so we can do beautiful parking
                .lineToConstantHeading(new Vector2d(23.5 * 2.5, -23.5*.5))
                //move linear slide down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideBottomAbsolute();
                }).waitSeconds(.2)
                //NOW Park using camera
                .lineToConstantHeading(new Vector2d(23.5 * 2.5, parkY))

                /*
                //EXPERIMENTAL CODE FOR SECOND CYCLE
                // START OF CYCLE 2
                // REMOVE THIS FROM HERE IF WE KEEP LINEAR SLIDE UP TO AVOID BUMPING
                // move back to CONE 4
                .addTemporalMarker(() -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(5.6 * 2.54 * 10.0 * 4/5);
                })
                // then go to stack and turn 90 degrees
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 8, -23.5 - 3, Math.toRadians(-90)))
                // intake for a bit
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(-.3);
                }).waitSeconds(.2)
                // move an inch back in order to not bump into field barrier when going up
                .lineTo(new Vector2d(23.5 * 2.5 - 8, -23.5 - 1.5))
                // stop and move linear slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    //CHANGE: make it -10 mm down to intake
                    setSlideMMAbsolute(355);
                }).waitSeconds(.2)
                // go to the junction to drop
                // CHANGE: A little more forward
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 11.5, -23.5*.5, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(.5);
                }).waitSeconds(1.3)
                */

                .build();

        //cycle
        drive.followTrajectorySequence(traj);

    }
}

