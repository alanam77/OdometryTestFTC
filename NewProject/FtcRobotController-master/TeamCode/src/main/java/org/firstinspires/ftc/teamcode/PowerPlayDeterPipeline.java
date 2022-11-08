package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class PowerPlayDeterPipeline{
    PowerPlayDeterPipeline pipeline;
    public static class PowerPlayPipeline extends OpenCvPipeline {

        static final Scalar RED = new Scalar(255,0,0);
        static final Scalar GREEN = new Scalar(0,255,0);
        static final Scalar BLUE = new Scalar(0,0,255);

        static final int REGION_W = 20;
        static final int REGION_H = 20;

        static final Point mnPoint = new Point(160,120);

        Point regionApt = new Point(mnPoint.x, mnPoint.y);
        Point regionBpt = new Point(mnPoint.x + REGION_W, mnPoint.y + REGION_H);

        Mat RGB = new Mat();
        Mat regionB, regionR, regionG;
        Mat Cb = new Mat();
        Mat Cr = new Mat();
        Mat Cg = new Mat();
        public int valB, valR, valG;

        public void inputToRGB(Mat input){
            RGB = input;
            Core.extractChannel(RGB,Cb,2);
            Core.extractChannel(RGB,Cr, 0);
            Core.extractChannel(RGB,Cg, 1);
        }



        @Override
        public void init(Mat initFrame){
            inputToRGB(initFrame);
            regionB = Cb.submat(new Rect(regionApt, regionBpt));
            regionR = Cr.submat(new Rect(regionApt, regionBpt));
            regionG = Cg.submat(new Rect(regionApt, regionBpt));
        }

        @Override
        public Mat processFrame(Mat input){
            inputToRGB(input);
            valB = (int) Core.mean(regionB).val[0];
            valR = (int) Core.mean(regionR).val[0];
            valG = (int) Core.mean(regionG).val[0];
            Imgproc.rectangle(input, regionApt, regionBpt,GREEN, 2);
            return input;
        }
    }
}
