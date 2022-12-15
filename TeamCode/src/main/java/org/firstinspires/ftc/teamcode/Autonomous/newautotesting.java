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
@Autonomous(name = "newautotesting", group = "Autonomous")
public class newautotesting extends Main {

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

        // forward by 52" -> 132 cm
        encoderForward(132.08, .5);

        // left strafe by 12" -> 30.48 cm
        encoderStrafe(-30.48, .5);

        setSlideMaxAbsolute(.6);

        // forward to align -> 3"
        encoderForward(7.62, .3);

        // outtake
        moveServo(3000, 1);

        // go back to dis-align -> 3"
        encoderForward(-7.62, .3);

        setSlideBottomAbsolute(.75);

        // right strafe by 12" -> 30.48 cm
        encoderStrafe(30.48, .5);

        // turn to the stack
        turnHeading(90, .4f);



        /*
        // moving forward to middle
        encoderForward(124, .4);

        telemetry.addLine("bruh");
        telemetry.update();


        // strafe infront of junction
        encoderStrafe(-31, .56);

        telemetry.addLine("yey it got pass");
        telemetry.update();

        encoderForward(6, .5);

        // raise slide to high
        setSlideMaxAbsolute(.6);

        // outtake
        moveServo(3000, 1);

        // bring slide back down
        setSlideMMAbsolute(123, .5);

        // strafe to  near stacks
        encoderStrafe(100, .5);

        // turn to face stacks
        turnHeading(90, .4f);

        //move forward to attach to stacks
        encoderForward(15, .5);


        // EDIT FROM RIGHT: strafe is going left to go back
        // EDIT FROM RIGHT: now we have to go
        // 12" + 24" * ENUM -> 30.48cm + 60.96cm * ENUM
        //encoderForward(-30.48 - 60.96 * (2-pposition) - 1.5, 0.4);

        // reset the linear slides to the position
        // wait for it to go slightly down due to gravity (so that it's smoother when it pulls down with power)
        //setSlideBottomAbsolute(.75);

*/

    }

}
