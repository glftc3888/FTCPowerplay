package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Main;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoGeneral extends Main {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initRobot();
        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(23.5 * 1, 0))
                .addTemporalMarker(() -> {
                    setSlideTicksAbsolute(3000);
                })
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(23.5 * 2 - 8, -23.5 * .5 - 8, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    setPowerServo(1);
                }).waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    setPowerServo(0);
                    setSlideBottomAbsolute();
                })
                .build();

        drive.followTrajectorySequence(traj);

    }

}
