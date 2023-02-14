package org.firstinspires.ftc.teamcode.auto;
/* * Copyright (c) 2021 OpenFTC Team
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
 * SOFTWARE.*/




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class AprilTagAutonomousInitDetectionExampleTest extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    private int autoStart = 0;
    private int autoMode = 0;
    private int initialDelay = 0;
    private int barcodeValue = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    double ONE = 1;
    double TWO = 2;
    double THREE = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);



/*         * The INIT-loop:
         * This REPLACES waitForStart!*/


        while (!isStarted() && !isStopRequested())
        {

            //create variable to store visual information
            String[] display = {"Blue-Left", "Blue-Right", "Red-Left",
                    "Red-Right", "Park"};

            //for loop to allow selection of autonomous situation
            for (boolean buttonPressed = false; !gamepad2.a && !opModeIsActive() && !isStopRequested();) {

                //change starting line
                if (gamepad2.dpad_left && autoStart > 0 && !buttonPressed) {
                    autoStart--;
                    buttonPressed = true;
                } else if (gamepad2.dpad_right && autoStart < 3 && !buttonPressed) {
                    autoStart++;
                    buttonPressed = true;
                    //change automode
                }
else if (gamepad1.dpad_down && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad1.dpad_up && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }

                //change initial delay
                else if (gamepad2.dpad_down && initialDelay > 0 && !buttonPressed) {
                    initialDelay--;
                    buttonPressed = true;
                } else if (gamepad2.dpad_up && initialDelay < 25 && !buttonPressed) {
                    initialDelay++;
                    buttonPressed = true;
                }
                //choose which autonomous mode
                else if (gamepad2.left_bumper && autoMode > 0 && !buttonPressed) {
                    autoMode--;
                    buttonPressed = true;
                } else if (gamepad2.right_bumper && autoMode < 1 && !buttonPressed) {
                    autoMode++;
                    buttonPressed = true;
                }
                //choose barcode position
                else if (gamepad2.square && barcodeValue > 0 && !buttonPressed) {
                    barcodeValue--;
                    buttonPressed = true;
                } else if (gamepad2.circle && barcodeValue < 2 && !buttonPressed) {
                    barcodeValue++;
                    buttonPressed = true;
                }

                //wait until buttons are not pressed
                else if (buttonPressed && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up &&
                        !gamepad2.dpad_down && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.square && !gamepad2.circle)
                    buttonPressed = false;



                //output telemetry
                telemetry.addData("Auto Start (Right/Left Dpad)", display[autoStart]);
                telemetry.addData("Auto Mode (Bumpers)", display[autoMode + 4]);
                telemetry.addData("initialDelay (Up/Down Dpad)", initialDelay);
                telemetry.addData("BarcodeValue", barcodeValue);



                telemetry.update();

            }
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ONE || tag.id == TWO || tag.id == THREE)
                    {
                        tagOfInterest = tag;
                        tagFound = true;

                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            if (tagOfInterest.id == ONE)
                barcodeValue = 1;
            if (tagOfInterest.id == TWO)
                barcodeValue = 2;
            if (tagOfInterest.id == THREE)
                barcodeValue = 3;



            telemetry.update();
            sleep(20);
        }

/*         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.*/






// Update the telemetry

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //TODO add roadrunner movement

        telemetry.update();


        
        int blueLeft = 0;
        int blueRight = 1;
        int redLeft = 2;
        int redRight = 3;
        int park = 0;


/* Actually do something useful */

        if(autoStart == 0 && tagOfInterest.id == ONE && autoMode == park){
            //trajectory
        } else if (autoStart == 0 && tagOfInterest.id == TWO && autoMode == park) {
            //trajectory
        } else if (autoStart == 0 && tagOfInterest.id == THREE && autoMode == park) {
            //trajectory
        } else if (autoStart == 1 && tagOfInterest.id == ONE && autoMode == park) {

        } else if (autoStart == 1 && tagOfInterest.id == TWO && autoMode == park) {

        } else if (autoStart == 1 && tagOfInterest.id == THREE && autoMode == park) {

        } else if (autoStart == 2 && tagOfInterest.id == ONE && autoMode == park) {

        } else if (autoStart == 2 && tagOfInterest.id == TWO && autoMode == park) {

        } else if (autoStart == 2 && tagOfInterest.id == THREE && autoMode == park) {

        } else if (autoStart == 3 && tagOfInterest.id == ONE && autoMode == park) {

        } else if (autoStart == 3 && tagOfInterest.id == TWO && autoMode == park) {

        } else if (autoStart == 3 && tagOfInterest.id == THREE && autoMode == park) {

        }





/* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

//        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
