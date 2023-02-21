package org.firstinspires.ftc.teamcode.Autonomous.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlideHolder {
    private static DcMotor linear_slide = null;

    public static void setLinearSlide(DcMotor linear_slide) {
        LinearSlideHolder.linear_slide = linear_slide;
    }

    public DcMotor getLinearSlide(){
        return linear_slide;
    }
}