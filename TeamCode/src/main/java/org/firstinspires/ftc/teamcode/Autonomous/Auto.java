/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.LeoOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LeoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Waiting for start");
        telemetry.update();
        // Send success signal
        telemetry.addData("Status", "Success!");
        telemetry.update();

        // Wait for start
        waitForStart();
        initRobo();
        runtime.reset();

        linearY(5, 0.1);

        /*initRobo();
        cam = new CompVision(hardwareMap, 1);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(path == -1){
            cam.printDebugCam1(telemetry);
            cam.printStats(telemetry);
            path = cam.getPath();
            telemetry.addLine("Path chosen: " + path);
            telemetry.update();
        }

        //cam.changePipeLine(0, 1);
        sleep(100);

        /*linearY(12, 0.2);
        turnHeading(-90, 0.2);
        linearY(5, 0.2);
        turnHeading(20, 0.2);
*/
        //runAutoPath(path);
        // grabCube();

/*
        closeClaw();
        moveArm(200, 0.5);
        sleep(100);
        turnHeading(-10, 0.2);
        linearY(9, 0.2);
        turnHeading(-45, 0.2);
        linearY(4, 0.2);
        turnHeading(-70, 0.2);
        linearY(5, 0.2);

        turnHeading(-90, 0.2);
        linearY(8, 0.4);
        turnHeading(-100);
        linearY(2, 0.4);
        turnHeading(-130);
        resetClaw(300);
        turnHeading(-180);


 */
    }
}