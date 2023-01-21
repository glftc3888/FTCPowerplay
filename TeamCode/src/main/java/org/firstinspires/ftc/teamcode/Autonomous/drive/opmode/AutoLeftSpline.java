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
public class AutoLeftSpline extends Main {

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

        telemetry.addLine("READY!!!!!");
        telemetry.update();

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                // go to middle of next tile
                //.lineToConstantHeading(new Vector2d(23.5 * 1, 0))
                .lineTo(new Vector2d(23.5 * 2.5 - 5, 0))
                //move back to the correct position
                .lineToConstantHeading(new Vector2d(23.5 * 1, 0 - 5.5))
                // move slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideTicksAbsolute(3000);
                })
                // move to align to pole
                //TO CHANGE
                .lineToLinearHeading(new Pose2d(23.5 * 2 - 8, (23.5 * .5 - 12) * -1, Math.toRadians(90 * -1)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(1);
                }).waitSeconds(1)
                //right height for CONE 5
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    setSlideMMAbsolute(5.6 * 2.54 * 10.0 - 30);
                })
                // go back a bit
                .lineTo(new Vector2d(23.5 * 2 - 8 + 2, (23.5 * .5 - 12) * -1))

                // now position yourself - START OF CYCLE
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 10, 0 * -1, Math.toRadians(-90 * -1)))
                //GOING TO STACK - CHANGE FOR AUTORIGHT
                //CHANGE: 2" to the right
                // then go forward
                .lineTo(new Vector2d(23.5 * 2.5 - 8 + 2, (-23.5 - 3) * -1))
                // intake for a bit
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(-.3);
                }).waitSeconds(.2)
                // move an inch back in order to not bump into field barrier when going up
                .lineTo(new Vector2d(23.5 * 2.5 - 8, (-23.5 - 1.5) * -1))
                // stop and move linear slide up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                    //CHANGE: make it -10 mm down to intake
                    setSlideMMAbsolute(350);
                }).waitSeconds(.2)
                // go to the junction to drop
                // CHANGE: A little more forward
                .lineToLinearHeading(new Pose2d(23.5 * 2.5 - 7, (-23.5*.5) * -1, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(.5);
                }).waitSeconds(1.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                }).waitSeconds(.2)
                //go back to center so we can do beautiful parking
                .lineToConstantHeading(new Vector2d(23.5 * 2.5 - 8, parkY))
                //move linear slide down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setSlideBottomAbsolute();
                }).waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(0, parkY))
                //NOW Park using camera
                //.lineToConstantHeading(new Vector2d(23.5 * 2.5, parkY))
                /*
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPowerServo(0);
                }).waitSeconds(20)
                */
                .build();

        while(opModeIsActive() || !isStopRequested()) {
            //cycle
            drive.followTrajectorySequence(traj);
            break;
        }

    }
}

