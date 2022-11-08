/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")
//@Disabled
public class ConceptTensorFlowObjectDetectionWebcamSample extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 384.5 ;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                            if(recognition.getLabel() == LABELS[0]){        //1.Bolt

                                break;
                            }
                            else if(recognition.getLabel() == LABELS[1]){   //2.Bulb

                                break;
                            }
                            else{                                           //3.Panel

                                break;
                            }
                        }
                        telemetry.update();
                    }
                    break;
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void Forward(double Power, double DesiredAngle, double Inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = FR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newLeftTarget2 = BL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget2 = BR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftTarget);
            FR.setTargetPosition(newRightTarget);
            BL.setTargetPosition(newLeftTarget2);
            BR.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                double YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                if (YawAngle < (DesiredAngle - .5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A negative YawAngle is a clockwise rotation for the robot.
                    FL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    FR.setPower(Power);
                    BL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BR.setPower(Power);
                } else if (YawAngle > (DesiredAngle + .5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A positive YawAngle is a counter-clockwise rotation for the robot.
                    FL.setPower(Power);
                    FR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BL.setPower(Power);
                    BR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                } else {
                    FL.setPower(Power);
                    FR.setPower(Power);
                    BL.setPower(Power);
                    BR.setPower(Power);
                }
                telemetry.addData("Angle is", YawAngle);
                telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        FL.getCurrentPosition(),
                        FR.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            sleep(50);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Turn off RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void Reverse(double Power, double DesiredAngle, double Inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = FR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newLeftTarget2 = BL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget2 = BR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            FL.setTargetPosition(-newLeftTarget);
            FR.setTargetPosition(-newRightTarget);
            BL.setTargetPosition(-newLeftTarget2);
            BR.setTargetPosition(-newRightTarget2);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                double YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                if (YawAngle > (DesiredAngle + .5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A negative YawAngle is a clockwise rotation for the robot.
                    FL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    FR.setPower(Power);
                    BL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BR.setPower(Power);
                } else if (YawAngle < (DesiredAngle - .5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A positive YawAngle is a counter-clockwise rotation for the robot.
                    FL.setPower(Power);
                    FR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BL.setPower(Power);
                    BR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                } else {
                    FL.setPower(Power);
                    FR.setPower(Power);
                    BL.setPower(Power);
                    BR.setPower(Power);
                }
                telemetry.addData("Angle is", YawAngle);
                telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        FL.getCurrentPosition(),
                        FR.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            sleep(50);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);


            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Turn off RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void leftStrafe(double Power, double DesiredAngle, double Inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = FR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newLeftTarget2 = BL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget2 = BR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftTarget);
            FR.setTargetPosition(-newRightTarget);
            BL.setTargetPosition(-newLeftTarget2);
            BR.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                double YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                if (YawAngle > (DesiredAngle + 0.5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A negative YawAngle is a clockwise rotation for the robot.
                    FL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BL.setPower(Power);
                    FR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BR.setPower(Power);
                } else if (YawAngle < (DesiredAngle - 0.5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A positive YawAngle is a counter-clockwise rotation for the robot.
                    FL.setPower(Power);
                    BL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    FR.setPower(Power);
                    BR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                } else {
                    FL.setPower(Power);
                    FR.setPower(Power);
                    BL.setPower(Power);
                    BR.setPower(Power);
                }
                telemetry.addData("Angle is", YawAngle);
                telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        FL.getCurrentPosition(),
                        FR.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            sleep(50);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            // Turn off RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void RightStrafe(double Power, double DesiredAngle, double Inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = FR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newLeftTarget2 = BL.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightTarget2 = BR.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

            FL.setTargetPosition(-newLeftTarget);
            FR.setTargetPosition(newRightTarget);
            BL.setTargetPosition(newLeftTarget2);
            BR.setTargetPosition(-newRightTarget2);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                double YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                if (YawAngle < (DesiredAngle - 0.5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A negative YawAngle is a clockwise rotation for the robot.
                    FL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BL.setPower(Power);
                    FR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    BR.setPower(Power);
                } else if (YawAngle > (DesiredAngle + 0.5)) {
                    //Assumes we are looking at the robot from the center of the field and leftDrive & rightDrive are against the wall.
                    //A positive YawAngle is a counter-clockwise rotation for the robot.
                    FL.setPower(Power);
                    BL.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                    FR.setPower(Power);
                    BR.setPower(Power - ((abs(DesiredAngle - YawAngle)) / 25));
                } else {
                    FL.setPower(Power);
                    FR.setPower(Power);
                    BL.setPower(Power);
                    BR.setPower(Power);
                }
                telemetry.addData("Angle is", YawAngle);
                telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget2, newRightTarget2);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        FL.getCurrentPosition(),
                        FR.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            sleep(50);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Turn(double DesiredAngle, double timeoutS) {
        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < timeoutS) {

                FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                if (YawAngle < DesiredAngle - 0.5) {
                    FL.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                    FR.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                    BL.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                    BR.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                } else if (YawAngle > DesiredAngle + 0.5) {
                    FL.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                    FR.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                    BL.setPower(((abs(DesiredAngle - YawAngle)) / 75));
                    BR.setPower(-((abs(DesiredAngle - YawAngle)) / 75));
                } else {
                    FL.setPower(0);
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                }
                telemetry.addData("Angle is", YawAngle);
                telemetry.addData("Motor Power is", "At %7f :%7f :%7f :%7f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
                // Display it for the driver.
                //  telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", DesiredAngle);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        FL.getCurrentPosition(),
                        FR.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();


            }
            sleep(50);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
