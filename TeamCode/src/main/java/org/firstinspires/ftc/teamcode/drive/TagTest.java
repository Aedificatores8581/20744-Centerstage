package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.colorDistance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        AprilTagProcessor tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(camera, tagProcessor);
        List<AprilTagDetection> detections;
        ColorRangeSensor x = hardwareMap.get(ColorRangeSensor.class, "sensor");
        colorDistance a = new colorDistance(x);
        while (!isStopRequested()) {
            detections = tagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                AprilTagPoseFtc pose = detection.ftcPose;
                telemetry.addLine("Detection ID: " + detection.id);
                telemetry.addLine("Distance: " + pose.range);
                telemetry.addLine("Bearing: " + pose.bearing);
                telemetry.addLine("Angle: " + pose.yaw + "\n");
            }
            telemetry.update();
        }
    }
}