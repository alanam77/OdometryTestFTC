package org.firstinspires.ftc.teamcode;

public class XyhVector {
    public double x;
    public double y;
    public double heading;
    public XyhVector(double xVal, double yVal, double headingVal){
        x = xVal;
        y = yVal;
        heading = headingVal;
    }
    public XyhVector(XyhVector vec){
        x = vec.x;
        y = vec.y;
        heading = vec.heading;
    }
}
