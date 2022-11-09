package org.firstinspires.ftc.teamcode.Autonomous;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.TeleOp.Main;

// code from previous year - encoders, camera, motors
// https://github.com/greasedlightning/FtcRobotController
@Autonomous(name = "TestAuto", group = "Autonomous")
public class Auto extends Main {

    // Declaration of global variables
    private ElapsedTime runtime = new ElapsedTime();

    private static DcMotor frontLeftMotor = null;
    private static DcMotor backLeftMotor = null;
    private static DcMotor frontRightMotor = null;
    private static DcMotor backRightMotor = null;
    private static CRServo leftServo = null;
    private static CRServo rightServo = null;

    private DcMotor LinearSlide = null;

    // will use later... This is for angle (see code from last year)
    BNO055IMU IMU;

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

    private static boolean extended = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        initRobot();

        SleeveDetection.ParkingPosition parkingPosition;
        SleeveDetection sleeveDetection = new SleeveDetection();

        // Wait for start
        waitForStart();
        runtime.reset();

        parkingPosition = sleeveDetection.getPosition();

        // LEFT, CENTER, RIGHT --> 0, 1, 2   = 'parkingPosition' object

        // AT THE END: 12 inches + 24*ENUM(0 1 OR 2)

        // DO THINGS -- BELOW

        encoderForward(66.0, 0.2);

        encoderStrafe(-110.0, 0.2);

        setSlideMMAbsolute(820, .6);

        encoderForward(2.0, 0.2);

        moveServo(800, 1);

        encoderForward(-2.0, 0.2);

        encoderStrafe(130, 0.2);

    }

}





