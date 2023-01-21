package org.firstinspires.ftc.teamcode.Autonomous;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.Main;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// code from previous year - encoders, camera, motors
// https://github.com/greasedlightning/FtcRobotController
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "gyrotesting", group = "gyrotesting")
public class gyrotesting extends Main {

    // Declaration of global variables
    private ElapsedTime runtime = new ElapsedTime();

    // roughly 537.7, but ((((1+(46/17))) * (1+(46/11))) * 28) to be exact (on the site)
    // Link for motor:
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // Ticks Per Rotation (how many ticks in one full motor rotation)
    private static final double TPR = (1+(46/17)) * (1+(46/11)) * 28;// ticks per revolution
    // circumference of the pulley circle pulling the string in linear slides
    private static final double CIRCUMFERENCE = 112; // in mm
    // DON'T USE THIS, IF IT'S TOO MUCH IT MIGHT BREAK THE LINEAR SLIDE
    private double MAX_LINEAR_SLIDE_EXTENSION = 976; // in mm
    private static final double RADIUS = 4.8; // in cm
    private static final double PI=3.1415926535;
    private static final double WHEEL_CIRCUMFERENCE = 2*PI*RADIUS;


    // camera setup
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        initRobot();
        runtime.reset();

        // sleep for a bit in order to wait for the camera to sense the color
        Thread.sleep(1000);

        turnHeading(180, .5f);

    }

}
