package org.firstinspires.ftc.teamcode.TeleOp;

// Import modules
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class Main extends LinearOpMode {

    // Declaration of global variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor LinearSlide = null;

    private double FORTY_FIVE_IN_RADS = Math.PI/4;
    private double TPR = 537.7; // ticks per revolution

    @Override
    public void runOpMode() throws InterruptedException {


        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        // Setup hardware
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "back_left");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right");

        LinearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");

        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        //when you're setting it up (in opMode)
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);

        // Wait for start
        waitForStart();
        runtime.reset();

        // Start of OpMode
        while (opModeIsActive()) {
            // Power up the motors using left and right sticks
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            double rightY = gamepad1.right_stick_y;


            frontLeftMotor.setPower(leftY - leftX - rightX);
            frontRightMotor.setPower(leftY + leftX + rightX);
            backRightMotor.setPower(leftY - leftX + rightX);
            backLeftMotor.setPower(leftY + leftX - rightX);
            telemetry.addData("Status", "in");
            telemetry.update();

            if (gamepad1.a) {
                setSlide(4000, .75);
            }
            if (gamepad1.b) {
                setSlide(0, 0);
            }
        }
    }


// Make sure the encoder cables are connected right, and the the forward/backward is in the right place

    //this is the function for setSlideTicks
    public void setSlide(int ticks, double power) throws InterruptedException{
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LinearSlide.setTargetPosition(ticks);

        LinearSlide.setPower(power);

        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(LinearSlide.isBusy()) {
        }

        //linearSlide.setPower(0);

        //linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
