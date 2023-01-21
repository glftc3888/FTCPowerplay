package org.firstinspires.ftc.teamcode.Autonomous;

// Import modules
import com.qualcomm.hardware.bosch.BNO055IMU;
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
@Autonomous(name = "newautotesting", group = "Autonomous")
public class newautotesting extends Main {

    // Declaration of global variables
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;


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
        imu = hardwareMap.get(BNO055IMU.class, "Gyro");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters); // init the gyro

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
        //Thread.sleep(1000);

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
        encoderForward(132.08, .9);

        // left strafe by 12" -> 30.48 cm
        encoderStrafe(-30.48-3, .9);

        setSlideMaxAbsolute();

        // forward to align -> 3"
        encoderForward(7.62, .3);

        // outtake
        moveServo(1000, 1);

        // go back to dis-align -> 3"
        encoderForward(-7.62, .3);

        setSlideBottomAbsolute();

        // right strafe by 12" -> 30.48 cm
        encoderStrafe(30.48+3, .5);

        // turn to the stack
        turnPID(-90);

        //go up a little bit
        setSlideMMAbsolute(152 - 20);

        encoderStrafe(-5, .3);

        // go forward towards stack
        encoderForward(50.4 + 10, .7);

        // intake cone
        moveServo(1000,-1);

        //up the slide so we don't knock down stack
        setSlideMMAbsolute(152 + (int)(7*2.54));

        //going back
        encoderForward(-20 * 2.54, .5);

        //turning to low junc
        turnPID(-90);



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
