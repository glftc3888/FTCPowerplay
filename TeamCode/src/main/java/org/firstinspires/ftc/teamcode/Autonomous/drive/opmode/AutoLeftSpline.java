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

//        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-34.95, -64.45, Math.toRadians(90.00)))
//                .splineTo(new Vector2d(-35.65, -21.95), Math.toRadians(89.05))
//                .splineTo(new Vector2d(-27.92, -3.86), Math.toRadians(45.00))
//                .splineTo(new Vector2d(-41.62, -12.64), Math.toRadians(180.00))
//                .splineTo(new Vector2d(-57.60, -11.77), Math.toRadians(180.00))
//                .splineTo(new Vector2d(-27.40, -11.77), Math.toRadians(-1.80))
//                .splineTo(new Vector2d(-23.53, -5.62), Math.toRadians(90.00))
//                .splineTo(new Vector2d(-26.34, -11.77), Math.toRadians(180.00))
//                .splineTo(new Vector2d(-58.48, -11.24), Math.toRadians(181.38))
//                .splineTo(new Vector2d(-28.10, -11.59), Math.toRadians(-4.04))
//                .splineTo(new Vector2d(-23.36, -6.32), Math.toRadians(90.00))
//                .splineTo(new Vector2d(-26.69, -11.94), Math.toRadians(180.00))
//                .splineTo(new Vector2d(-59.18, -11.94), Math.toRadians(180.00))
//                .splineTo(new Vector2d(-23.18, -11.77), Math.toRadians(0.00))
//                .splineTo(new Vector2d(-23.53, -4.57), Math.toRadians(90.00))
//                .build();
//        drive.setPoseEstimate(untitled0.start());

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-35.82, -64.45, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-12.47, -56.20, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(-11.59, -13.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(-24.41, -9.31, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-38.99, -11.77, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-62.17, -11.94))
                .lineTo(new Vector2d(-41.62, -12.64))
                .lineToSplineHeading(new Pose2d(-24.06, -9.83, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-38.81, -12.12, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-62.87, -11.77, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-41.62, -12.12))
                .lineToSplineHeading(new Pose2d(-24.41, -9.48, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-38.99, -12.12, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-62.34, -12.12, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-41.97, -12.12))
                .lineToSplineHeading(new Pose2d(-24.76, -9.48, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-39.16, -12.47, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-62.69, -12.29, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-42.15, -12.29))
                .lineToSplineHeading(new Pose2d(-24.06, -9.66, Math.toRadians(90.00)))

                .addSpatialMarker(new Vector2d(-12, -20), () -> {
                    setSlideMaxAbsolute();
                })


//                .addSpatialMarker(new Vector2d(-34.95, -16.16), () -> {
//
//                })


                .build();

        drive.setPoseEstimate(untitled0.start());

        while(opModeIsActive() && !isStopRequested()) {

            drive.followTrajectorySequence(untitled0);

            break;
        }

    }
}

