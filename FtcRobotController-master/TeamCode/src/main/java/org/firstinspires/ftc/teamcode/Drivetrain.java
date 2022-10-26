package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain extends Thread {
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    public DcMotor LeftP = null;
    public DcMotor RightP = null;
    public DcMotor Aux = null;

    private LinearOpMode op = null;
    private HardwareMap mapH = null;

    public Drivetrain(HardwareMap hm, LinearOpMode OpM){
        mapH = hm;
        op = OpM;

        FL = mapH.get(DcMotor.class, "FL");
        FR = mapH.get(DcMotor.class, "FR");
        BL = mapH.get(DcMotor.class, "BL");
        BR = mapH.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftP = FL;
        RightP = FR;
        Aux = BL;

        resetEnc();

    }
    public void resetEnc(){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void stopMotors(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    final static double L = 20; //The dist. between the two encoder pods facing the same direction
    final static double B = 11; //distance between the midpoint of x1 and x2 pods, and the position of y1 pod
    final static double R = 3; // Radius of the wheel of the pod
    final static double N = 8192; //Encoder Resolution
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    public int currRpos = 0;
    public int currLpos = 0;
    public int currApos = 0;

    public int oldRpos = 0;
    public int oldLpos = 0;
    public int oldApos = 0;

    public XyhVector startPos = new XyhVector(100, 100, 0);
    public XyhVector pos =  new XyhVector(startPos);

    public void Odometry(){
        oldRpos = currRpos;
        oldLpos = currLpos;
        oldApos = currApos;

        currRpos = RightP.getCurrentPosition();
        currLpos = LeftP.getCurrentPosition();
        currApos = Aux.getCurrentPosition();

        int dn1 = currLpos - oldLpos;
        int dn2 = currRpos - oldRpos;
        int dn3 = currApos - oldApos;

        double dtheta = cm_per_tick * (dn2 - dn1) / L;
        double dx = cm_per_tick * (dn2 + dn1) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L);

        double theta = pos.heading + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) - dy * Math.cos(theta);
        pos.heading += dtheta;
    }

    public void goToPos(double x, double y, double h, double power, double errorMargin){
        double distToX = x - pos.x;
        double distToY = y - pos.y;

        double distance = Math.hypot(distToX,distToY);

        while(op.opModeIsActive() && distance > errorMargin){
            distance = Math.hypot(distToX,distToY);

            distToX = x - pos.x;
            distToY = y - pos.y;

            double robotMoveAngle = Math.toDegrees(Math.atan2(distToX,distToY));

            double robotXComp = calX(robotMoveAngle,power);
            double robotYComp = calY(robotMoveAngle,power);
            double pivCorrect = h - pos.heading;


            double r = Math.hypot(robotXComp,robotYComp);
            double robotAngle = Math.atan2(robotYComp,robotXComp) - Math.PI / 4;
            double rightX = pivCorrect;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            FL.setPower(v1);
            FR.setPower(v2);
            BL.setPower(v3);
            BR.setPower(v4);

        }
    }
    public double calX(double desiredAngle, double speed){
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    public double calY(double desiredAngle, double speed){
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    public void run(){
        Odometry();
    }
}
