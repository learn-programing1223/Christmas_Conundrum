package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class cameraPipeline extends OpenCvPipeline {

    public static double whitePixels;
    double[] targetOrange = {232, 164, 56};
    double[] targetWhite = {255, 255, 255};

    double[] red = {255, 0, 0, 1};
    double[] blue = {0, 0, 255, 1};
    double wpc = 0;

    @Override
    public Mat processFrame(Mat input) {
        Mat output = input.clone();
//        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGBA2RGB);
        Size dimensions = input.size();
        double height = dimensions.height;
        double width = dimensions.width;

        Rect box = new Rect(new Point(width/3, height/3),new Point(2*width/3, 2*height/3));
        Imgproc.rectangle(output, box, new Scalar(4,233,78),3,8);

        for(int i= (int)Math.round(height/3); i < (int)Math.round(2*height/3); i++){
            for (int j = (int)Math.round(width/3); j < (int)Math.round(2*width/3); j++){
                double[] pixelColor = input.get(i,j);
                if(compareColor(targetOrange, pixelColor)){
                    output.put(i,j,red);
                }
                else if(compareColor(targetWhite, pixelColor)){
                    output.put(i,j,blue);
                    wpc++;
                }
            }
        }
        whitePixels=wpc;
        return output;
    }

    private static boolean compareColor(double[] targetColor, double[] pixelColor) {
        boolean output = true;
        for(int i = 0; i < 3; i++) {
            output = output && pixelColor[i] < targetColor[i] * (1 + 0.2) && pixelColor[i] > targetColor[i] * (1 - 0.2);
        }
        return output;
    }

}
