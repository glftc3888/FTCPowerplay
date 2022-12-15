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
@Autonomous(name = "AutoLeft", group = "Autonomous")
public class AutoLeft extends Main {

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

        // setup for the camera
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

        // no need for waitForStart() with while loop
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.addData("YEL: ", sleeveDetection.yelPercent());
            telemetry.addData("CYA: ", sleeveDetection.cyaPercent());
            telemetry.addData("MAG: ", sleeveDetection.magPercent());
            telemetry.update();
        }


        initRobot();
        runtime.reset();

        // sleep for a bit in order to wait for the camera to sense the color
        Thread.sleep(1000);

        // get parking position enum
        SleeveDetection.ParkingPosition parkingPosition;

        // camera code
        int pposition = -1;
        while (pposition == -1) {
            parkingPosition = sleeveDetection.getPosition();
            switch (parkingPosition) {
                case LEFT:
                    pposition = 0;
                    break;
                case CENTER:
                    pposition = 1;
                    break;
                case RIGHT:
                    pposition = 2;
                    break;
                case NOTHING: // default, keep at -1
                    pposition = -1;
                    break;
            }
        }

        telemetry.addLine(String.valueOf(pposition));
        telemetry.update();

        // LEFT, CENTER, RIGHT --> 0, 1, 2   = 'parkingPosition' object

        // AT THE END: 12 inches + 24*ENUM(0 1 OR 2)

        // DO THINGS -- BELOW

        // 30" forward -> 76.2cm - 6cm (too much)   /morning of comp/ changed from  .3 to .5
        encoderForward(76.2-4, .5);

        // 36" left -> 91.44 cm
        // EDIT FROM RIGHT: strafe is going right now  /morning of comp/ changed from .2 to .4
        encoderStrafe(91.44 + 1.5, .4);

        // change to go to max junction later
        setSlideMaxAbsolute(.65);

        // in case we need to align forward
        // encoderForward(2.0, 0.2);

        // move forward to align with the pole
        encoderForward(6, .2);

        // move servo to outtake
        moveServo(3000, 1);

        // move backwards to be at the center
        encoderForward(-6, .2);

        // move backward just in case (not to bump into junction)
        encoderForward(-5, 0.2);

        //start of new autonoumous

        /*
        encoderStrafe(152.4, .4);

        encoderForward(60.96, .4);

        //
        encoderStrafe(-30.48, .4);
        //
        encoderForward(60.96, .4);
        //
        turnHeading(180, .4);
        //
        encoderForward(121.92, .4);
        // go to 5 cone pos
        setSlideMMAbsolute(220,.6);

        encoderForward(1,.4);

        moveServo(-3000, -.5);

        setSlideMMAbsolute(100,.6);

        encoderForward(-122.92,.4);

        encoderStrafe(30.48, .4);

        setSlideMaxAbsolute(.5);

        moveServo(3000, .5);

        setSlideBottomAbsolute(.4);

        encoderStrafe(-30.48, .4);
        */



        // reset the linear slides to the position
        // wait for it to go slightly down due to gravity (so that it's smoother when it pulls down with power)
        setSlideBottomAbsolute(.4);

        // EDIT FROM RIGHT: strafe is going left to go back
        // EDIT FROM RIGHT: now we have to go
        // 12" + 24" * ENUM -> 30.48cm + 60.96cm * ENUM
        //chang to encider strafe for old auto
        encoderStrafe(-30.48 - 60.96 * (2-pposition) - 1.5, 0.4);




    }

}