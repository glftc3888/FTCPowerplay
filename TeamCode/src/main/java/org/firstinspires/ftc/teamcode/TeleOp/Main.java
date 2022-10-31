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

    private double FORTY_FIVE_IN_RADS = Math.PI/4;

    @Override
    public void runOpMode() {
        double x1 = 0; //left or right lol
        double y1 = 0; // front or back lol
        double cosine45 = Math.cos(FORTY_FIVE_IN_RADS);
        double sine45 = Math.sin(FORTY_FIVE_IN_RADS);

        double x2 = 0;
        double y2 = 0;
        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        // Setup hardware
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "back_left");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "front_right");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
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

                /*
                //if moving joystick then spin
                frontLeftMotor.setPower(-spin);
                backRightMotor.setPower(spin);

                frontRightMotor.setPower(-spin);
                backLeftMotor.setPower(spin);

            }
            else {
                //normal driving if nothing is happening with the right joystick
                y1 = -gamepad1.left_stick_y;
                x1 = gamepad1.right_stick_x;
                //rotate 45 counter clockwise lol
                y2 = y1 * cosine45 + x1 * sine45;
                x2 = x1 * cosine45 - y1 * sine45;

                frontLeftMotor.setPower(x2);
                backRightMotor.setPower(x2);

                frontRightMotor.setPower(y2);
                backLeftMotor.setPower(y2);
            }
            */
//            telemetry.addData("Status", "Nothing Pressed");
//            telemetry.update();
        }
    }
}
