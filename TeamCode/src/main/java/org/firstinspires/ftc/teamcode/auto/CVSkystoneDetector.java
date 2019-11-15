package org.firstinspires.ftc.teamcode.auto;

import android.util.Pair;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ConfigUser;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.stream.IntStream;

public class CVSkystoneDetector extends ConfigUser<CVSkystoneDetector.SkystoneDetectionConfigSchema> {

  public static class SkystoneDetectionConfigSchema {
    public String cameraRotation;
    public int streamWidth, streamHeight;
    public int stoneWidth, stoneHeight;

    public int leftStoneMidX, leftStoneMidY;
    public int yellowHue; // 48 / 2
  }

  private OpenCvCamera phoneCam;
  private SkystoneCVPipeline pipeline;

  public CVSkystoneDetector(HardwareMap hardwareMap) {
    super("cvSkystoneDetector.properties", new SkystoneDetectionConfigSchema());

    int cameraMonitorViewId = hardwareMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    pipeline = new SkystoneCVPipeline();
  }

  public void open() {
    phoneCam.openCameraDevice();
    phoneCam.setPipeline(pipeline);
    phoneCam.startStreaming(config.streamWidth, config.streamHeight,
        OpenCvCameraRotation.valueOf(config.cameraRotation));
  }

  public void close() {
    phoneCam.stopStreaming();
  }

  public Pair<StonePosition, Double> estimatedSkystonePosition() {
    return pipeline.estimatedSkystonePosition();
  }

  private class SkystoneCVPipeline extends OpenCvPipeline {
    private double[] lastReadProbabilities = new double[StonePosition.values().length];

    Pair<StonePosition, Double> estimatedSkystonePosition() {
      int maxProbabilityIndex = maxIndex(lastReadProbabilities);
      return Pair.create(StonePosition.values()[maxProbabilityIndex],
          averageVariance(maxProbabilityIndex, lastReadProbabilities));
    }

    private int maxIndex(double[] array) {
      double max = Double.NEGATIVE_INFINITY;
      int index = -1;

      for (int i = 0; i < array.length; ++i) {
        if (array[i] > max) {
          max = array[i];
          index = i;
        }
      }
      return index;
    }

    private double averageVariance(int maxIndex, double[] array) {
      return IntStream.range(0, array.length)
          .filter(it -> it != maxIndex)
          // indices without the max
          .mapToDouble(index -> array[index])
          // elements without the max
          .map(elem -> Math.abs(elem - array[maxIndex]))
          // differences
          .average().orElse(0);
    }

    @Override
    public Mat processFrame(Mat input) {
      for (int i = 0; i < lastReadProbabilities.length; ++i) {
        StonePosition position = StonePosition.values()[i];

        Rect frame = stoneFrame(position);
        Scalar color = averageHsvColor(input, frame);

        Imgproc.rectangle(input, frame, new Scalar(255, 255, 255));
        lastReadProbabilities[i] = probabilityThatItIsSkystone(color);
      }

      return input;
    }

    private Rect stoneFrame(StonePosition position) {
      int x = config.leftStoneMidX - config.stoneWidth / 2;
      final int y = config.leftStoneMidY - config.stoneHeight / 2;

      x += position.offsetLeft * config.stoneWidth;

      return new Rect(x, y, config.stoneWidth, config.stoneHeight);
    }

    private Scalar averageHsvColor(Mat input, Rect frame) {
      Mat mat = input.submat(frame);
      Mat hsvMat = mat.clone();

      Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_BGR2HSV);
      Scalar color = Core.mean(hsvMat);

      mat.release();
      hsvMat.release();

      return color;
    }

    private double probabilityThatItIsSkystone(Scalar hsvColor) {
      // CV's ranges for HSV are [0, 179], [0, 255], [0,255]
      double H = hsvColor.val[0], S = hsvColor.val[1], V = hsvColor.val[2];

      double hDiff = Math.abs(H - config.yellowHue);
      if (hDiff > 90) hDiff = 180 - hDiff;

      return (hDiff/90 + 2 - S/255 - V/255) / 3;
    }
  }
}
