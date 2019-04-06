
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
//unused import java.io.Console;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
//unused import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.Mat;
import org.opencv.core.*;
//unused import org.opencv.core.Core.*;
//unused import org.opencv.features2d.FeatureDetector;
//unused import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
//unused import org.opencv.objdetect.*;

// TEAM 4150 imports

// TEAM 4150 end imports

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  // eclipse @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  // eclipse @SuppressWarnings("MemberName")
  public static class SwitchedCameraConfig {
    public String name;
    public String key;
  };

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
  public static List<VideoSource> cameras = new ArrayList<>();

  // --------team 4150 global data

  private static cls4150Timer objNoTargetDelayTimer = new cls4150Timer(3.0D);
  private static double dblLastGoodTargetAngle = 0.0D;
  private static double dblLastGoodRobotAngle = 0.0D;

  private static clsNetTblEntryInfo objVisRobotAngle = new clsNetTblEntryInfo("/robot/angle");

  private static clsNetTblEntryInfo objVisTargRobotAngleOut = new clsNetTblEntryInfo("/vision/RobotAngleFeedback");
  private static clsNetTblEntryInfo objVisTargAngle = new clsNetTblEntryInfo("/vision/TargetAngle");
  private static clsNetTblEntryInfo objVisTargFound = new clsNetTblEntryInfo("/vision/TargetFound");
  private static clsNetTblEntryInfo objVisTargWatchDog = new clsNetTblEntryInfo("/vision/TargetWatchDog");
  private static clsNetTblEntryInfo objVisTargSelectedScore = new clsNetTblEntryInfo("/vision/SelectedScore");
  private static clsNetTblEntryInfo objVisTargHighestScore = new clsNetTblEntryInfo("/vision/Debug/HighestScore");
  private static clsNetTblEntryInfo objVisTargTapeCount = new clsNetTblEntryInfo("/vision/Debug/TapeCount");
  private static clsNetTblEntryInfo objVisTargTargetCount = new clsNetTblEntryInfo("/vision/Debug/TargetCount");
  private static clsNetTblEntryInfo objVisTargTargetCombos = new clsNetTblEntryInfo("/vision/Debug/TargetCombos");

  private static clsNetTblEntryInfo objVisDebugFoundTarg = new clsNetTblEntryInfo("/vision/Debug/ReallyFoundTarg");
  private static clsNetTblEntryInfo objVisDebugIndex = new clsNetTblEntryInfo("/vision/Debug/TargetIndex");
  private static clsNetTblEntryInfo objVisDebugWidth = new clsNetTblEntryInfo("/vision/Debug/Width");
  private static clsNetTblEntryInfo objVisDebugHeight = new clsNetTblEntryInfo("/vision/Debug/Height");
  private static clsNetTblEntryInfo objVisDebugLeftIndex = new clsNetTblEntryInfo("/vision/Debug/LeftIndex");
  private static clsNetTblEntryInfo objVisDebugRightIndex = new clsNetTblEntryInfo("/vision/Debug/RightIndex");
  private static clsNetTblEntryInfo objVisDebugLeftAngle = new clsNetTblEntryInfo("/vision/Debug/LeftAngle");
  private static clsNetTblEntryInfo objVisDebugRightAngle = new clsNetTblEntryInfo("/vision/Debug/RightAngle");
  private static clsNetTblEntryInfo objVisDebugLeftScore = new clsNetTblEntryInfo("/vision/Debug/LeftScore");
  private static clsNetTblEntryInfo objVisDebugRightScore = new clsNetTblEntryInfo("/vision/Debug/RightScore");
  private static clsNetTblEntryInfo objVisDebugLeftRatio = new clsNetTblEntryInfo("/vision/Debug/LeftRatio");
  private static clsNetTblEntryInfo objVisDebugRightRatio = new clsNetTblEntryInfo("/vision/Debug/RightRatio");
  private static clsNetTblEntryInfo objVisDebugHeightRatioScore = new clsNetTblEntryInfo(
      "/vision/Debug/HeightRatioScore");
  private static clsNetTblEntryInfo objVisDebugWidthRatioScore = new clsNetTblEntryInfo(
      "/vision/Debug/WidthRatioScore");
  private static clsNetTblEntryInfo objVisDebugOverallRatioScore = new clsNetTblEntryInfo(
      "/vision/Debug/AreaRatioScore");
  private static clsNetTblEntryInfo objVisDebugHeightOffsetScore = new clsNetTblEntryInfo(
      "/vision/Debug/HeightOffsetScore");
  private static clsNetTblEntryInfo objVisDebugOverallScore = new clsNetTblEntryInfo("/vision/Debug/OverallScore");
  private static clsNetTblEntryInfo objVisDebugCenterPixel = new clsNetTblEntryInfo("/vision/Debug/CenterPixel");
  private static clsNetTblEntryInfo objVisDebugCenterOffsetPixel = new clsNetTblEntryInfo(
      "/vision/Debug/CenterOffsetPixel");
  private static clsNetTblEntryInfo objVisDebugCenterOffsetAngle = new clsNetTblEntryInfo(
      "/vision/Debug/CenterOffsetAngle");

  private static clsNetTblEntryInfo objVisHSVHueLow = new clsNetTblEntryInfo("/vision/HSV/HueLow");
  private static clsNetTblEntryInfo objVisHSVHueHigh = new clsNetTblEntryInfo("/vision/HSV/HueHigh");
  private static clsNetTblEntryInfo objVisHSVSatLow = new clsNetTblEntryInfo("/vision/HSV/SatLow");
  private static clsNetTblEntryInfo objVisHSVSatHigh = new clsNetTblEntryInfo("/vision/HSV/SatHigh");
  private static clsNetTblEntryInfo objVisHSVValLow = new clsNetTblEntryInfo("/vision/HSV/ValLow");
  private static clsNetTblEntryInfo objVisHSVValHigh = new clsNetTblEntryInfo("/vision/HSV/ValHigh");
  // --------values to write back for debug... once this works, remove...
  private static clsNetTblEntryInfo objVisDebugHSVHueLow = new clsNetTblEntryInfo("/vision/Debug/HSV/HueLow");
  private static clsNetTblEntryInfo objVisDebugHSVHueHigh = new clsNetTblEntryInfo("/vision/Debug/HSV/HueHigh");
  private static clsNetTblEntryInfo objVisDebugHSVSatLow = new clsNetTblEntryInfo("/vision/Debug/HSV/SatLow");
  private static clsNetTblEntryInfo objVisDebugHSVSatHigh = new clsNetTblEntryInfo("/vision/Debug/HSV/SatHigh");
  private static clsNetTblEntryInfo objVisDebugHSVValLow = new clsNetTblEntryInfo("/vision/Debug/HSV/ValLow");
  private static clsNetTblEntryInfo objVisDebugHSVValHigh = new clsNetTblEntryInfo("/vision/Debug/HSV/ValHigh");

  // --------color filter values
  private static double dblHueLow = 70.66187050359712D;
  private static double dblHueHigh = 128.13651877133107D;
  private static double dblSatLow = 21.98741007194246D;
  private static double dblSatHigh = 109.0D;
  private static double dblValLow = 206.98741007194246D;
  private static double dblValHigh = 255.0D;

  // -------team 4150 end global data

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read single switched camera configuration.
   */
  public static boolean readSwitchedCameraConfig(JsonObject config) {
    SwitchedCameraConfig cam = new SwitchedCameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read switched camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement keyElement = config.get("key");
    if (keyElement == null) {
      parseError("switched camera '" + cam.name + "': could not read key");
      return false;
    }
    cam.key = keyElement.getAsString();

    switchedCameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  // eclipse @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    if (obj.has("switched cameras")) {
      JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
      for (JsonElement camera : switchedCameras) {
        if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    CameraServer inst = CameraServer.getInstance();
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer server = inst.startAutomaticCapture(camera);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Start running the switched camera.
   */
  public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
    System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
    MjpegServer server = CameraServer.getInstance().addSwitchedCamera(config.name);

    NetworkTableInstance.getDefault().getEntry(config.key).addListener(event -> {
      if (event.value.isDouble()) {
        int i = (int) event.value.getDouble();
        if (i >= 0 && i < cameras.size()) {
          server.setSource(cameras.get(i));
        }
      } else if (event.value.isString()) {
        String str = event.value.getString();
        for (int i = 0; i < cameraConfigs.size(); i++) {
          if (str.equals(cameraConfigs.get(i).name)) {
            server.setSource(cameras.get(i));
            break;
          }
        }
      }
    }, EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    return server;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline {
    public int val;

    @Override
    public void process(Mat mat) {
      val += 1;
    }
  }

  // ================TEAM 4150 data structure starts

  // --------data structure to hold data for each piece of tape.
  // --------for now this is just data. There are no methods.
  @SuppressWarnings("unused")
  private static class clsTapeInfo {
    // ----- for now there is just a bounding rectangle.
    // ----- The data structure is created like this in case
    // ----- there is a way to read a trapezoid. For now
    // ----- the top and bottom X are the same and the
    // ----- right and left Y are the same.
    // ----- NOTE: OpenCV uses the TOP LEFT as coord 0,0. Numbers increase down and
    // left
    public int intTopLeftX = 0;
    public int intTopLeftY = 0;
    public int intTopRightX = 0;
    public int intTopRightY = 0;
    public int intBotLeftX = 0;
    public int intBotLeftY = 0;
    public int intBotRightX = 0;
    public int intBotRightY = 0;
    public double dblAngle = 0;
    public double dblHeight = 0.0D;
    public double dblWidth = 0.0D;
    public double dblSizeRatioScore = 0.0; // ratio of Height/Width score for this tape.
  }

  // --------data structure to hold data for each potential target
  // --------which consists of a pair of pieces of tape
  private static class clsTargetInfo {
    public clsTapeInfo objLeftTape; // object for left tape piece
    public int intLeftTapeIndex = 0; // index for left tape
    public clsTapeInfo objRightTape; // object for right tape piece
    public int intRightTapeIndex = 0; // index for right tape
    public double dblHeight = 0.0D; // overall Height of target
    public double dblWidth = 0.0D; // overall Width of target
    public double dblCenterX = 0.0D; // pixel center of
    public double dblCenterPixelOffsetX = 0.0D; // X offset from center (pixels)
    public double dblCenterAngleOffsetX = 0.0D; // X offset from center (pseudo angle) -- not TRUE angle
    public double dblSizeRatioScore = 0.0D; // score for overall size ratio
    public double dblHeightRatioScore = 0.0D; // score for ratio of the Height of the two tapes
    public double dblWidthRatioScore = 0.0D; // score for ratio of the Width of the two tapes
    public double dblHeightOffsetScore = 0.0D; // score for the y axis offset of the two tapes
    public double dblOverallScore = 0.0D; // average of the 6 individual scores
  }

  // ================TEAM 4150 data structure ends

  /**
   * Main.
   */
  public static void main(String... args) {

    // --------the only calling argument is the configuration file name.
    if (args.length > 0) {
      configFile = args[0];
    }

    // --------read configuration
    if (!readConfig()) {
      return;
    }

    // --------start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }
    // ============= team 4150 code
    objVisRobotAngle.SetTable(ntinst);

    objVisTargRobotAngleOut.SetTable(ntinst);
    objVisTargAngle.SetTable(ntinst);
    objVisTargFound.SetTable(ntinst);
    objVisTargWatchDog.SetTable(ntinst);
    objVisTargSelectedScore.SetTable(ntinst);
    objVisTargHighestScore.SetTable(ntinst);
    objVisTargTapeCount.SetTable(ntinst);
    objVisTargTargetCount.SetTable(ntinst);
    objVisTargTargetCombos.SetTable(ntinst);

    objVisDebugFoundTarg.SetTable(ntinst);
    objVisDebugIndex.SetTable(ntinst);
    objVisDebugWidth.SetTable(ntinst);
    objVisDebugHeight.SetTable(ntinst);
    objVisDebugLeftIndex.SetTable(ntinst);
    objVisDebugRightIndex.SetTable(ntinst);
    objVisDebugLeftAngle.SetTable(ntinst);
    objVisDebugRightAngle.SetTable(ntinst);
    objVisDebugLeftScore.SetTable(ntinst);
    objVisDebugRightScore.SetTable(ntinst);
    objVisDebugLeftRatio.SetTable(ntinst);
    objVisDebugRightRatio.SetTable(ntinst);
    objVisDebugHeightRatioScore.SetTable(ntinst);
    objVisDebugWidthRatioScore.SetTable(ntinst);
    objVisDebugOverallRatioScore.SetTable(ntinst);
    objVisDebugHeightOffsetScore.SetTable(ntinst);
    objVisDebugOverallScore.SetTable(ntinst);
    objVisDebugCenterPixel.SetTable(ntinst);
    objVisDebugCenterOffsetPixel.SetTable(ntinst);
    objVisDebugCenterOffsetAngle.SetTable(ntinst);

    objVisHSVHueLow.SetTable(ntinst);
    objVisHSVHueHigh.SetTable(ntinst);
    objVisHSVSatLow.SetTable(ntinst);
    objVisHSVSatHigh.SetTable(ntinst);
    objVisHSVValLow.SetTable(ntinst);
    objVisHSVValHigh.SetTable(ntinst);

    objVisDebugHSVHueLow.SetTable(ntinst);
    objVisDebugHSVHueHigh.SetTable(ntinst);
    objVisDebugHSVSatLow.SetTable(ntinst);
    objVisDebugHSVSatHigh.SetTable(ntinst);
    objVisDebugHSVValLow.SetTable(ntinst);
    objVisDebugHSVValHigh.SetTable(ntinst);

    // --------ensure these are persistent on the robot
    objVisHSVHueLow.SetPersistent(dblHueLow);
    objVisHSVHueHigh.SetPersistent(dblHueHigh);
    objVisHSVSatLow.SetPersistent(dblSatLow);
    objVisHSVSatHigh.SetPersistent(dblSatHigh);
    objVisHSVValLow.SetPersistent(dblValLow);
    objVisHSVValHigh.SetPersistent(dblValHigh);

    // ======== end team 4150 code

    // --------start cameras
    for (CameraConfig config : cameraConfigs) {
      cameras.add(startCamera(config));
    }

    // --------start switched cameras
    for (SwitchedCameraConfig config : switchedCameraConfigs) {
      startSwitchedCamera(config);
    }

    // ======================== TEAM 4150 code starts here

    // --------start image processing on camera X if present
    // --------this is a little crude..
    VideoSource camera = null;
    for (VideoSource c : cameras) {
      if (c.getName().equals("VISION_CAM")) {
        System.out.println("------> Found VISION_CAM");
        camera = c;
        break;
      }
    }
    // orig-now-grip VisionThread visionThread = new VisionThread(cameras.get(0),
    // orig-now-grip new MyPipeline(), pipeline -> {
    // orig-now-grip // do something with pipeline results
    // orig-now-grip });
    // something like this for GRIP:
    if (camera != null) {
      VisionThread visionThread = new VisionThread(camera,
          // orig-now-grip new GripPipeline(), pipeline -> {
          new GRIPVision(), pipeline -> {

            // -------- do something with pipeline results
            // debug System.out.println("--------------------pipe run--------------");

            // --------define variables
            double dblRobotAngle = 0.0D; // current angle of robot --- read from robot
            // unused double dblTargetAngle;
            boolean boolFoundTarg = false; // did we find a target
            boolean boolReallyFoundTarg = false; // did we really find a target (not off delay value...)
            double dblSelectedOffsetAngle = 9000.0D; // offset of selected target
            double dblHighestScore = 0.0D;
            double dblSelectedScore = 0.0D;
            int intSelectedTargInx = 0; // selected index into target array
            int intHighestScoreInx = 0; // index of highest score (for reporting when nothing found)
            int intTapeCount = 0;
            int intTargCntAboveMinScore = 0;
            int intTargetCount = 0;

            // debug
            // System.out.println("--------------------beforeNetworkTableGet--------------");

            // --------get the color filter values
            // --------use last good value ( or initial value as default )
            dblHueLow = objVisHSVHueLow.GetDouble(dblHueLow);
            dblHueHigh = objVisHSVHueHigh.GetDouble(dblHueHigh);
            dblSatLow = objVisHSVSatLow.GetDouble(dblSatLow);
            dblSatHigh = objVisHSVSatHigh.GetDouble(dblSatHigh);
            dblValLow = objVisHSVValLow.GetDouble(dblValLow);
            dblValHigh = objVisHSVValHigh.GetDouble(dblValHigh);
            // --------set grip module values.
            pipeline.setHSVValues(dblHueLow, dblHueHigh, dblSatLow, dblSatHigh, dblValLow, dblValHigh);

            // --------get the current robot position angle -- read network tables
            // ----orig ---- dblRobotAngle =
            // ntinst.getEntry("/SmartDashboard/angle").getDouble(0);
            dblRobotAngle = objVisRobotAngle.GetDouble(dblLastGoodRobotAngle);
            dblLastGoodRobotAngle = dblRobotAngle; // save for times of bad communication.

            // unused dblTargetAngle = dblRobotAngle; //-- if nothing found return robot
            // angle...(or leave unchanged!)

            // --------is there something to process.
            // debug
            // System.out.println("--------------------beforeContourCheck--------------");
            if (!pipeline.findContoursOutput().isEmpty()) {

              ArrayList<MatOfPoint> objLocContourOutput; // local copy of contour output

              clsTapeInfo[] objTapeInfo = new clsTapeInfo[200]; // THIS IS BAD PROGRAMMING PRACTICE!!!!
              clsTargetInfo[] objTargetInfo = new clsTargetInfo[19900]; // THIS IS BAD PROGRAMMING PRACTICE!!!! (200 *
                                                                        // 199) / ( 2 * 1 )

              int j; // loop index
              int j1; // loop index

              // --------get a copy of the output. The local copy helps to prevent
              // synchronization errors.
              // synchronized( imgLock ) { // TODO - add thread sync
              // debug
              // System.out.println("--------------------beforeContoursOutput--------------");
              objLocContourOutput = pipeline.findContoursOutput();
              // } // TODO - add end thread sync

              // --------find out how many contours were found
              // debug System.out.println("--------------------before getting
              // count--------------");
              intTapeCount = objLocContourOutput.size();

              // --------need at least two tape strips to make a target. If less, don't bother
              // processing
              // debug
              // System.out.println("--------------------beforeCreatingBoundingRect--------------"
              // + intTapeCount);
              if (intTapeCount >= 2) {

                // --------fill in the tape info for each contour
                for (j = 0; j < intTapeCount; j++) {

                  objTapeInfo[j] = new clsTapeInfo(); // create instance of object in array
                  Rect objTapeRectangle; // bounding rectangle for tape contour
                  MatOfPoint2f objPoint2f;
                  RotatedRect objTapeRotatedRectange; // rotated bounding rectangle...
                  // debug System.out.println("------------------- processing tape - " + j);
                  try {
                    // --------get the information for this contour
                    objTapeRectangle = Imgproc.boundingRect(objLocContourOutput.get(j));
                    // TODO: Replace bounding RECT with minAreaRect to get rotated rectange!!!
                    // --------for now just collect angle and report.
                    objPoint2f = new MatOfPoint2f(objLocContourOutput.get(j).toArray());
                    objTapeRotatedRectange = Imgproc.minAreaRect(objPoint2f);

                    // debug System.out.println(objTapeRectangle);

                    // --------fill in the data structure for this piece of tape (contour)
                    // ----- NOTE: OpenCV uses the TOP LEFT as coord 0,0. Numbers increase down and
                    // left
                    // ----- changed the Y calculations to fix direction of increae. (originally
                    // used bottom left origin)
                    objTapeInfo[j].intTopLeftX = objTapeRectangle.x;
                    objTapeInfo[j].intTopLeftY = objTapeRectangle.y;

                    objTapeInfo[j].intTopRightX = objTapeRectangle.x + objTapeRectangle.width;
                    objTapeInfo[j].intTopRightY = objTapeRectangle.y;

                    objTapeInfo[j].intBotLeftX = objTapeRectangle.x;
                    objTapeInfo[j].intBotLeftY = objTapeRectangle.y + objTapeRectangle.height;

                    objTapeInfo[j].intBotRightX = objTapeRectangle.x + objTapeRectangle.width;
                    objTapeInfo[j].intBotRightY = objTapeRectangle.y + objTapeRectangle.width;

                    objTapeInfo[j].dblWidth = (double) objTapeRectangle.width;
                    objTapeInfo[j].dblHeight = (double) objTapeRectangle.height;

                    objTapeInfo[j].dblAngle = objTapeRotatedRectange.angle + 90.0D; // -- make straight up = 0.

                    // --------Width = 3.32, Height = 5.82
                    objTapeInfo[j].dblSizeRatioScore = 100.0D
                        * (1.0D - Math.abs(objTapeInfo[j].dblHeight / objTapeInfo[j].dblWidth / 1.75301D - 1.0D));
                    // old (1.0 - Math.abs(objTapeInfo[j].dblHeight / objTapeInfo[j].dblWidth -
                    // 1.75301));

                    // debug System.out.println("--------------------in bounding rect--------------"
                    // + j);
                  } // end try
                  catch (Exception e) {
                    // debug System.out.println(e);
                  } finally {
                  } // end finally

                } // end loop

                // debug System.out.println("--------------------starting to process pairs of
                // tape--------------");
                // --------create pairs of tape to make potential targets

                // --------set the number of potential targets processed to zero.
                intTargetCount = 0;

                // --------create potential targets by processing all combinations of pairs of
                // tape.
                for (j = 0; j < intTapeCount - 1; j++) {
                  for (j1 = j + 1; j1 < intTapeCount; j1++) {

                    // -------- create the target data structure instance
                    objTargetInfo[intTargetCount] = new clsTargetInfo();

                    // -------fill in the tape pieces. determine which piece is left and right
                    if (objTapeInfo[j].intBotLeftX < objTapeInfo[j1].intBotLeftX) {
                      objTargetInfo[intTargetCount].objLeftTape = objTapeInfo[j];
                      objTargetInfo[intTargetCount].intLeftTapeIndex = j;
                      objTargetInfo[intTargetCount].objRightTape = objTapeInfo[j1];
                      objTargetInfo[intTargetCount].intRightTapeIndex = j1;
                    } else {
                      objTargetInfo[intTargetCount].objLeftTape = objTapeInfo[j1];
                      objTargetInfo[intTargetCount].intLeftTapeIndex = j1;
                      objTargetInfo[intTargetCount].objRightTape = objTapeInfo[j];
                      objTargetInfo[intTargetCount].intRightTapeIndex = j;
                    }
                    // --------calc overall Width and Height... Calculation is generic just in case
                    // --------one contour covers the entire width or height.
                    objTargetInfo[intTargetCount].dblWidth = Math.max(
                        objTargetInfo[intTargetCount].objRightTape.intBotRightX,
                        objTargetInfo[intTargetCount].objLeftTape.intBotRightX)
                        - Math.min(objTargetInfo[intTargetCount].objLeftTape.intBotLeftX,
                            objTargetInfo[intTargetCount].objRightTape.intBotLeftX);
                    // --------switched bottom and top.. had direction of increase backwards...
                    objTargetInfo[intTargetCount].dblHeight = Math.max(
                        objTargetInfo[intTargetCount].objRightTape.intBotRightY,
                        objTargetInfo[intTargetCount].objLeftTape.intBotRightY)
                        - Math.min(objTargetInfo[intTargetCount].objLeftTape.intTopRightY,
                            objTargetInfo[intTargetCount].objRightTape.intTopRightY);

                    // --------calc overall size ratio score
                    // --------Width = 14.64, Height = 5.82
                    // debug
                    // System.out.println("--------------------beforeCalculatingIndividualScores--------------");
                    objTargetInfo[intTargetCount].dblSizeRatioScore = 100.0D * (1.0D - Math
                        .abs(objTargetInfo[intTargetCount].dblHeight / objTargetInfo[intTargetCount].dblWidth / 0.39754D
                            - 1.0D));
                    // -- old (1.0 - Math.abs(objTargetInfo[intTargetCount].dblHeight /
                    // objTargetInfo[intTargetCount].dblWidth - 0.39754));

                    // --------calc pair Height ratio score
                    objTargetInfo[intTargetCount].dblHeightRatioScore = 100.0D
                        * (1.0D - Math.abs(objTargetInfo[intTargetCount].objLeftTape.dblHeight
                            / objTargetInfo[intTargetCount].objRightTape.dblHeight - 1.0D));

                    // --------calc pair Width ratio score
                    objTargetInfo[intTargetCount].dblWidthRatioScore = 100.0D
                        * (1.0D - Math.abs(objTargetInfo[intTargetCount].objLeftTape.dblWidth
                            / objTargetInfo[intTargetCount].objRightTape.dblWidth - 1.0D));

                    // --------calc y average offset score (this one may not be worth much....)
                    // --------PARENS were incorrect. Changed.
                    objTargetInfo[intTargetCount].dblHeightOffsetScore = 100.0D
                        * (1.0D - Math.abs(((objTargetInfo[intTargetCount].objRightTape.intBotLeftY
                            + objTargetInfo[intTargetCount].objRightTape.intTopLeftY) * 0.5D
                            - (objTargetInfo[intTargetCount].objLeftTape.intBotLeftY
                                + objTargetInfo[intTargetCount].objLeftTape.intTopLeftY) * 0.5D)
                            / objTargetInfo[intTargetCount].dblHeight));

                    // --------calc overall score = average of
                    // -------- tape1 ratio score
                    // -------- tape2 ratio score
                    // -------- overall size ratio score
                    // -------- Width comparison ratio score
                    // -------- Height comparison ratio score
                    // -------- Height differential ratio score
                    // debug
                    // System.out.println("--------------------beforeCalculatingdblOverallScores--------------");
                    objTargetInfo[intTargetCount].dblOverallScore = (objTargetInfo[intTargetCount].objLeftTape.dblSizeRatioScore
                        + objTargetInfo[intTargetCount].objRightTape.dblSizeRatioScore
                        + objTargetInfo[intTargetCount].dblSizeRatioScore
                        + objTargetInfo[intTargetCount].dblWidthRatioScore
                        + objTargetInfo[intTargetCount].dblHeightRatioScore
                        + objTargetInfo[intTargetCount].dblHeightOffsetScore) / 6.0D;

                    // --------calc center X of target
                    objTargetInfo[intTargetCount].dblCenterX = (Math.max(
                        objTargetInfo[intTargetCount].objRightTape.intBotRightX,
                        objTargetInfo[intTargetCount].objLeftTape.intBotRightX)
                        + Math.min(objTargetInfo[intTargetCount].objLeftTape.intBotLeftX,
                            objTargetInfo[intTargetCount].objRightTape.intBotLeftX))
                        * 0.5;
                    // --------calc pixel offset
                    // --------image width in pixels = 320
                    objTargetInfo[intTargetCount].dblCenterPixelOffsetX = objTargetInfo[intTargetCount].dblCenterX
                        - 320.0D * 0.5D;

                    // --------calc angle offset (note - for now this is very simple and not quite
                    // correct. It doesn't
                    // --------account for distance away from the camera....)
                    // --------camera field of view is 44 degrees TODO - Camera field of view angle.
                    // Use experimental data or lookup in labview routines. )
                    objTargetInfo[intTargetCount].dblCenterAngleOffsetX = objTargetInfo[intTargetCount].dblCenterPixelOffsetX
                        * 44.0D / 320.0D;

                    // --------increment target array index
                    // --------NOTE: Since this increments after each iteration, the ending result
                    // is the number of potential
                    // --------targets processed.
                    intTargetCount++;

                  } // end j1
                } // end j

                // --------for all the targets, find the one with the smallest absolute center X
                // -------- offset that meets the minimum score.
                // debug
                // System.out.println("--------------------beforeCheckingForMinimumScore--------------");
                intTargCntAboveMinScore = 0;
                dblHighestScore = 0.0D;
                dblSelectedOffsetAngle = 9000D; // set the initial offset angle very high to start with.

                // --------loop through all potential targets. Find the one we want..
                for (j = 0; j < intTargetCount; j++) {

                  // -------for debugging and general info, find the highest score.
                  if (objTargetInfo[j].dblOverallScore > dblHighestScore) {
                    dblHighestScore = objTargetInfo[j].dblOverallScore;
                    intHighestScoreInx = j;
                  }

                  // -------only consider targets whose score is above a minimum. This could be
                  // lowered
                  // -------if needed. If it is too low, the robot will chase ghosts...
                  // -------also require potential target to be a minimum size. (approx 9 feet
                  // away)
                  // -------perhaps add a test for the lowest individual score to make certain it
                  // isn't too small.
                  if (objTargetInfo[j].dblOverallScore >= 75.0D && objTargetInfo[j].dblWidth >= 40.0D
                      && objTargetInfo[j].dblHeight >= 18.0D) {
                    intTargCntAboveMinScore++; // ------- for debug increment how many targets met the min score.
                    boolFoundTarg = true; // ------- indicate we found at least one qualifying target.
                    boolReallyFoundTarg = true; // ------- really found a target (not off delay value)
                    objNoTargetDelayTimer.startTimer(); // --------start the delay timer.

                    // --------if this target is closer to the center, then select it.
                    if (Math.abs(objTargetInfo[j].dblCenterAngleOffsetX) < Math.abs(dblSelectedOffsetAngle)) {
                      dblSelectedOffsetAngle = objTargetInfo[j].dblCenterAngleOffsetX;
                      dblLastGoodTargetAngle = dblRobotAngle + dblSelectedOffsetAngle;
                      dblSelectedScore = objTargetInfo[j].dblOverallScore;
                      intSelectedTargInx = j;
                    }
                  }
                } // --- end j

                // --------debug -- write extra info to robot
                if (boolReallyFoundTarg) { // -- selected target
                  objVisDebugFoundTarg.WriteBoolean(boolReallyFoundTarg);
                  objVisDebugIndex.WriteDouble((double) intSelectedTargInx);
                  objVisDebugWidth.WriteDouble(objTargetInfo[intSelectedTargInx].dblWidth);
                  objVisDebugHeight.WriteDouble(objTargetInfo[intSelectedTargInx].dblHeight);
                  objVisDebugLeftIndex.WriteDouble(objTargetInfo[intSelectedTargInx].intLeftTapeIndex);
                  objVisDebugRightIndex.WriteDouble(objTargetInfo[intSelectedTargInx].intRightTapeIndex);
                  objVisDebugLeftAngle.WriteDouble(objTargetInfo[intSelectedTargInx].objLeftTape.dblAngle);
                  objVisDebugRightAngle.WriteDouble(objTargetInfo[intSelectedTargInx].objRightTape.dblAngle);
                  objVisDebugLeftScore.WriteDouble(objTargetInfo[intSelectedTargInx].objLeftTape.dblSizeRatioScore);
                  objVisDebugRightScore.WriteDouble(objTargetInfo[intSelectedTargInx].objRightTape.dblSizeRatioScore);
                  objVisDebugLeftRatio.WriteDouble(objTargetInfo[intSelectedTargInx].objLeftTape.dblHeight
                      / objTargetInfo[intSelectedTargInx].objLeftTape.dblWidth);
                  objVisDebugRightRatio.WriteDouble(objTargetInfo[intSelectedTargInx].objRightTape.dblHeight
                      / objTargetInfo[intSelectedTargInx].objRightTape.dblWidth);
                  objVisDebugHeightRatioScore.WriteDouble(objTargetInfo[intSelectedTargInx].dblHeightRatioScore);
                  objVisDebugWidthRatioScore.WriteDouble(objTargetInfo[intSelectedTargInx].dblWidthRatioScore);
                  objVisDebugOverallRatioScore.WriteDouble(objTargetInfo[intSelectedTargInx].dblSizeRatioScore);
                  objVisDebugHeightOffsetScore.WriteDouble(objTargetInfo[intSelectedTargInx].dblHeightOffsetScore);
                  objVisDebugOverallScore.WriteDouble(objTargetInfo[intSelectedTargInx].dblOverallScore);
                  objVisDebugCenterPixel.WriteDouble(objTargetInfo[intSelectedTargInx].dblCenterX);
                  objVisDebugCenterOffsetPixel.WriteDouble(objTargetInfo[intSelectedTargInx].dblCenterPixelOffsetX);
                  objVisDebugCenterOffsetAngle.WriteDouble(objTargetInfo[intSelectedTargInx].dblCenterAngleOffsetX);
                } else { // --- highest non selected score
                  objVisDebugFoundTarg.WriteBoolean(boolReallyFoundTarg);
                  objVisDebugIndex.WriteDouble((double) intHighestScoreInx);
                  objVisDebugWidth.WriteDouble(objTargetInfo[intHighestScoreInx].dblWidth);
                  objVisDebugHeight.WriteDouble(objTargetInfo[intHighestScoreInx].dblHeight);
                  objVisDebugLeftIndex.WriteDouble(objTargetInfo[intHighestScoreInx].intLeftTapeIndex);
                  objVisDebugRightIndex.WriteDouble(objTargetInfo[intHighestScoreInx].intRightTapeIndex);
                  objVisDebugLeftAngle.WriteDouble(objTargetInfo[intHighestScoreInx].objLeftTape.dblAngle);
                  objVisDebugRightAngle.WriteDouble(objTargetInfo[intHighestScoreInx].objRightTape.dblAngle);
                  objVisDebugLeftScore.WriteDouble(objTargetInfo[intHighestScoreInx].objLeftTape.dblSizeRatioScore);
                  objVisDebugRightScore.WriteDouble(objTargetInfo[intHighestScoreInx].objRightTape.dblSizeRatioScore);
                  objVisDebugLeftRatio.WriteDouble(objTargetInfo[intHighestScoreInx].objLeftTape.dblHeight
                      / objTargetInfo[intHighestScoreInx].objLeftTape.dblWidth);
                  objVisDebugRightRatio.WriteDouble(objTargetInfo[intHighestScoreInx].objRightTape.dblHeight
                      / objTargetInfo[intHighestScoreInx].objRightTape.dblWidth);
                  objVisDebugHeightRatioScore.WriteDouble(objTargetInfo[intHighestScoreInx].dblHeightRatioScore);
                  objVisDebugWidthRatioScore.WriteDouble(objTargetInfo[intHighestScoreInx].dblWidthRatioScore);
                  objVisDebugOverallRatioScore.WriteDouble(objTargetInfo[intHighestScoreInx].dblSizeRatioScore);
                  objVisDebugHeightOffsetScore.WriteDouble(objTargetInfo[intHighestScoreInx].dblHeightOffsetScore);
                  objVisDebugOverallScore.WriteDouble(objTargetInfo[intHighestScoreInx].dblOverallScore);
                  objVisDebugCenterPixel.WriteDouble(objTargetInfo[intHighestScoreInx].dblCenterX);
                  objVisDebugCenterOffsetPixel.WriteDouble(objTargetInfo[intHighestScoreInx].dblCenterPixelOffsetX);
                  objVisDebugCenterOffsetAngle.WriteDouble(objTargetInfo[intHighestScoreInx].dblCenterAngleOffsetX);

                }
              } // --- intTapeCount > 2
            } // --- no results

            // --------if target not found and timer not expired, set to good and use last
            // good angle...
            if (!boolFoundTarg) {
              if (!objNoTargetDelayTimer.isDone()) {
                boolFoundTarg = true;
                // System.out.println("--------------------------using delayed
                // bad-------------------------------");
              }
            }

            // --------write results to robot....
            // debug
            // System.out.println("--------------------beforeWritingResults--------------");
            objVisTargRobotAngleOut.WriteDouble(dblRobotAngle); // -- feed the robot angle back to dashboard for
                                                                // comparision.
            if (boolFoundTarg) {
              objVisTargAngle.WriteDouble(dblLastGoodTargetAngle);
            } else {
              objVisTargAngle.WriteDouble(dblRobotAngle);
            }
            objVisTargFound.WriteBoolean(boolFoundTarg);
            objVisTargWatchDog.WriteDouble(pipeline.wDog);
            objVisTargSelectedScore.WriteDouble(dblSelectedScore);
            // --------debug
            objVisTargHighestScore.WriteDouble(dblHighestScore);
            objVisTargTapeCount.WriteDouble((double) intTapeCount);
            objVisTargTargetCount.WriteDouble((double) intTargCntAboveMinScore);
            objVisTargTargetCombos.WriteDouble((double) intTargetCount);
            // --------more debug
            objVisDebugHSVHueLow.WriteDouble(pipeline.getHueLow());
            objVisDebugHSVHueHigh.WriteDouble(pipeline.getHueHigh());
            objVisDebugHSVSatLow.WriteDouble(pipeline.getSaturationLow());
            objVisDebugHSVSatHigh.WriteDouble(pipeline.getSaturationHigh());
            objVisDebugHSVValLow.WriteDouble(pipeline.getValueLow());
            objVisDebugHSVValHigh.WriteDouble(pipeline.getValueHigh());

          });
      // */

      // ======================== TEAM 4150 code ends here

      visionThread.start();
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}
