package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

// Single use per object
public class PIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetPos;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private boolean isAngle = true;

    public PIDController(double target, double p, double i, double d, boolean isAngle) {
        kP = p;
        kI = i;
        kD = d;
        targetPos = target;
        this.isAngle = isAngle;
    }

    public void setTargetPosition(double target){
        this.targetPos = target;
    }

    public double update(double currentPos) {
        double error = targetPos - currentPos;

        if (isAngle) {
            error %= 360;
            error += 360;
            error %= 360;
            if (error > 180) {
                error -= 360;
            }
        }

        // I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        double motorPower = 0.1 * Math.signum(error)
                + 0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);
        return motorPower;
    }

    public double getLastSlope() {
        return lastSlope;
    }
}