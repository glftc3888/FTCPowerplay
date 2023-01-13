package org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.TeleOp.Main;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "Odometry", group = "TeleOp")


public class Odometry extends Main{
    // Motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    // Encoder Ticks per revolution
    private final double TICKS_PER_REV = 537.7;
    // Wheel Circumference
    private final double WHEEL_CIRCUMFERENCE = 27.75;
    // Distance per tick
    private final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
    // Track the total distance for each wheel
    private double frontLeftDistance = 0;
    private double frontRightDistance = 0;
    private double backLeftDistance = 0;
    private double backRightDistance = 0;
    // Track the robot's position and heading
    private double x = 0;
    private double y = 0;
    private double heading = 0;
    // Target angle for turning
    private double targetAngle = 0;
    // Flag to indicate if the robot is currently turning
    private boolean isTurning = false;
    private TurnPIDController pid;


    public Odometry(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        pid = new TurnPIDController(0, 0.01, 0.000, 0.006);
    }

    public void update() {
        if (!isTurning) {
            // Get the number of ticks since the last update for each wheel
            int frontLeftTicks = frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition();
            int frontRightTicks = frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition();
            int backLeftTicks = backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition();
            int backRightTicks = backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition();

            // Update the total distance for each wheel
            frontLeftDistance += frontLeftTicks * DISTANCE_PER_TICK;
            frontRightDistance += frontRightTicks * DISTANCE_PER_TICK;
            backLeftDistance += backLeftTicks * DISTANCE_PER_TICK;
            backRightDistance += backRightTicks * DISTANCE_PER_TICK;

            // Calculate the average distance and delta heading
            double avgDistance = (frontLeftDistance + frontRightDistance + backLeftDistance + backRightDistance) / 4.0;
            double deltaHeading = (frontRightDistance - frontLeftDistance + backRightDistance - backLeftDistance) / (2 * WHEEL_CIRCUMFERENCE);

            // Update the robot's position and heading
            heading += deltaHeading;
            x += avgDistance * Math.cos(heading);
            y += avgDistance * Math.sin(heading);

            // Reset the target position for each wheel
            frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition());
            frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition());
            backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition());
            backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition());
        } else {
            turnToPID(targetAngle);
            if (Math.abs(targetAngle - getAbsoluteAngle()) <= 0.5 && pid.getLastSlope() <= 0.75) {
                isTurning = false;
                setMotorPower(0, 0, 0, 0);
            }
        }
    }

}



