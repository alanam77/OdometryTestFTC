package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="PPPipelineDetect", group="Linear Opmode")
public class PowerPlayAutoDetect extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 384.5 ;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    OpenCvWebcam webcam;
    PowerPlayDeterPipeline.PowerPlayPipeline pipeline;
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    @Override
    public void runOpMode(){
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PowerPlayDeterPipeline.PowerPlayPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        while(opModeInInit()){
            telemetry.addData("Blue value: ", pipeline.valB);
            telemetry.addData("Red value: ", pipeline.valR);
            telemetry.addData("Green value: ", pipeline.valG);
            telemetry.addData("Heading(Z): ", angles.firstAngle);
            telemetry.addData("Roll(Y): ", angles.secondAngle);
            telemetry.addData("Pitch(X): ", angles.thirdAngle);

            telemetry.update();
        }
        while(opModeIsActive()){
            if((pipeline.valG - pipeline.valR) < 20 && (pipeline.valG - pipeline.valB) < 20){           //Green Side
                //Dependent Upon Your Sleeve
            }
            else if(pipeline.valB > pipeline.valR && pipeline.valB > pipeline.valG){                    //Blue Side
                //Dependent Upon Your Sleeve
            }
            else{                                                                                       //Red Side
                //Dependent Upon Your Sleeve
            }
            break;
        }
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
