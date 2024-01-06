package org.firstinspires.ftc.teamcode;

import android.webkit.WebBackForwardList;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class ColorDetectionTest extends OpMode {
    OpenCvWebcam webcam1 = null;

    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
webcam1.setPipeline(new examplePipeline());

webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
    @Override
    public void onOpened() {
        webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
});
    }

    @Override
    public void loop() {

    }
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Double leftavgin;
        Double rightavgin;
        Mat outPut = new Mat();

        Rect leftRect = new Rect(1, 1, 319, 359);

        Rect rightRect = new Rect(320, 1, 319, 359);

        int color = 1;
        //red = 1, blue = 2

        /*
        Determine absolute position of marker and return which line the marker is on by testing two cases:
            -left and middle line visible
            -right and middle line visible
            *line positions are ordered as left is closest to back drop when observing from side*
         */
        boolean samplingLeft = false;
        boolean samplingRight = false;

        /*
        relative locations of the marker based on which side you are viewing it from used to determine absolute position
         */

        MarkerPosition markerPositionRight = MarkerPosition.UNKOWN;
        MarkerPosition markerPositionLeft = MarkerPosition.UNKOWN;
        MarkerPosition absoluteMarkerPosition = MarkerPosition.UNKOWN;

        //still need to implement when to turn on the sampling for the left and right side
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            input.copyTo(outPut);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, color);
            Core.extractChannel(rightCrop, rightCrop, color);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgin = leftavg.val[0];
            rightavgin = rightavg.val[0];
            int sensitivity = 10;

            //when viewing just left and middle lines
            if (samplingLeft) {
                //lower -> more sensitive to choosing left or right
                if (leftavgin - rightavgin < sensitivity && leftavgin - rightavgin > -sensitivity) {
                    //nothing detected
                    telemetry.addLine("Nothing or middle");
                    markerPositionLeft = MarkerPosition.UNKOWN;
                } else if (leftavgin > rightavgin) {
                    //on the left
                    telemetry.addLine("Left");
                    markerPositionLeft = MarkerPosition.LEFT;
                } else {
                    //on the right
                    telemetry.addLine("Right");
                    markerPositionLeft = MarkerPosition.RIGHT;
                }
            }

            //when viewing just right and middle lines
            if (samplingRight){
                //lower -> more sensitive to choosing left or right
                if (leftavgin - rightavgin < sensitivity && leftavgin - rightavgin > -sensitivity) {
                    //nothing detected
                    telemetry.addLine("Nothing or middle");
                    markerPositionRight = MarkerPosition.UNKOWN;
                } else if (leftavgin > rightavgin) {
                    //on the left
                    telemetry.addLine("Left");
                    markerPositionRight = MarkerPosition.LEFT;
                } else {
                    //on the right
                    telemetry.addLine("Right");
                    markerPositionRight = MarkerPosition.RIGHT;
                }
            }

            //determine absolute location

            //marker on left line
            if (markerPositionLeft == MarkerPosition.LEFT && markerPositionRight == MarkerPosition.UNKOWN){
                absoluteMarkerPosition = MarkerPosition.LEFT;
            }
            //marker on right line
            else if (markerPositionLeft == MarkerPosition.UNKOWN && markerPositionRight == MarkerPosition.RIGHT){
                absoluteMarkerPosition = MarkerPosition.RIGHT;
            }

            //marker on middle line
            else if (markerPositionLeft == MarkerPosition.RIGHT && markerPositionRight == MarkerPosition.LEFT){
                absoluteMarkerPosition = MarkerPosition.MIDDLE;
            }

            //not enough or conflicting readings
            else {
                absoluteMarkerPosition = MarkerPosition.UNKOWN;
            }
            return(outPut);

        }






    }
}