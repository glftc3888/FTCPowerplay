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

        // 30" forward -> 76.2cm - 6cm (too much)
        encoderForward(76.2-4, .3);

        // 36" left -> 91.44 cm
        // EDIT FROM RIGHT: strafe is going right now
        encoderStrafe(91.44, .2);

        // change to go to max junction later
        setSlideMaxAbsolute(.6);

        // experimental alternative version of autonomous
        // move forward to align with the pole
        encoderForward(15, .2);

        // bring the cone down into the pole (secure it)
        setSlideTicksAbsolute(1000, .5);

        // move servo to outtake
        moveServo(1500, .5);

        // finish moving back
        encoderForward(-15, .2);

        // straffe to correct position
        encoderStrafe(-30.48 - 60.96 * (2-pposition), 0.4);

        /*
        // move forward to align with the pole
        encoderForward(3, .2);

        // move servo to outtake
        moveServo(3000, .5);

        // move backwards to be at the center
        encoderForward(-3, .2);

        // move backward just in case (not to bump into junction)
        encoderForward(-2, 0.2);

        // EDIT FROM RIGHT: strafe is going left to go back
        // EDIT FROM RIGHT: now we have to go
        // 12" + 24" * ENUM -> 30.48cm + 60.96cm * ENUM
        encoderStrafe(-30.48 - 60.96 * (2-pposition), 0.4);
         */

        // reset the linear slides to the position
        // wait for it to go slightly down due to gravity (so that it's smoother when it pulls down with power)
        setSlideBottomAbsolute(.75);

    }

}





