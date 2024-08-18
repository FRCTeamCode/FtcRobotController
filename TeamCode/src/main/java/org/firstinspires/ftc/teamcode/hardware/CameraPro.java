package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

public class CameraPro {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private final Telemetry telemetry;
    private Boolean isTFOD, isTFODAprilTag;
    private static final String TFOD_MODEL_ASSET = "model_20231201_200350.tflite";//CDCenterStage.tflite
    private static final String[] CustomLabeles = {"BlueCube", "RedCube", "Pixel", "Z1", "Z2", "Z3"};

    public CameraPro(HardwareMap hardwareMap, Telemetry telemetry, boolean isTFOD, boolean isTFODAprilTag) {
        this.telemetry = telemetry;
        this.isTFOD = isTFOD;
        this.isTFODAprilTag = isTFODAprilTag;
        if (isTFOD) {
            if (isTFODAprilTag) {
                aprilTag = new AprilTagProcessor.Builder()
                        .setDrawAxes(false)
                        .setDrawCubeProjection(false)
                        .setDrawTagOutline(true)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                        .build();
                visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);
            } else {
                tfod = new TfodProcessor.Builder()
                        .setModelAssetName(TFOD_MODEL_ASSET)
                        .setModelLabels(CustomLabeles)
                        .setIsModelTensorFlow2(true)
                        .setIsModelQuantized(true)
                        .setModelInputSize(300)
                        .setModelAspectRatio(16.0 / 9.0)
                        .build();
                visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), tfod);
            }
        } else {
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        }
    }

    public void loop() {
        if (isTFOD&&!isTFODAprilTag) {
            telemetryTfod();
        } else {
            telemetryAprilTag();
        }
    }
    public void visionClose() {
        visionPortal.close();
    }

    public void telemetryAprilTag() {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections().size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections()) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
    public List<AprilTagDetection> currentDetections() {
        return aprilTag.getDetections();
    }

    public double[] getAprilTagIDData(int id) {//TO-DO fix
        double[] info = {-1.0, -1.0, -1.0, -1.0};
        if (id == 0) {
            if (currentDetections()!=null) {
                List<AprilTagDetection> detection = currentDetections();
                for (int i = 0; i < detection.size(); i++) {
                    if (detection.get(i) != null) {
                        info[0] = id;
                        if (detection.get(i).ftcPose != null) {
                            info[1] = detection.get(i).ftcPose.y;//forward-backward
                            info[2] = detection.get(i).ftcPose.x;//left-right
                            info[3] = detection.get(i).ftcPose.roll;//rotate
                        }
                        break;
                    }
                }
            }
        } else  {
            if (currentDetections()!=null) {
                List<AprilTagDetection> detection = currentDetections();
                for (int i = 0; i < detection.size(); i++) {
                    if (detection.get(i) != null) {
                        if (id == detection.get(i).id) {
                            info[0] = id;
                            if (detection.get(i).ftcPose != null) {
                                info[1] = detection.get(i).ftcPose.y;//forward-backward
                                info[2] = detection.get(i).ftcPose.x;//left-right
                                info[3] = detection.get(i).ftcPose.roll;//rotate
                            }
                            break;
                        }
                    }
                }
            }
        }
        return info;
    }



    public List<Recognition> currentRecognitions() {
        return tfod.getRecognitions();
    }
    public void telemetryTfod() {
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions().size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions()) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}
