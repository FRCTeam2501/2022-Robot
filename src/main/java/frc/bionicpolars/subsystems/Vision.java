package frc.bionicpolars.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static final int CAMERA_WIDTH = 320,
            CAMERA_HEIGHT = 240,
            CAMERA_FPS = 20;
    public enum Camera {
        INVALID,
        INTAKE,
        CLIMB
    }

    private UsbCamera climbCamera = new UsbCamera("Climb Camera", 0),
            intakeCamera = new UsbCamera("Intake Camera", 1);
    private MjpegServer videoServer = new MjpegServer("Video Server", 1185);
    private HttpCamera videoStream = new HttpCamera(
            "Video Stream",
            new String[]{ "http://roboRIO-2501-FRC.local/video/stream.mjpg", // mDNS
            "http://10.25.1.2/video/stream.mjpg", // Radio assigned IP
            "http://172.22.11.2/video/stream.mjpg" }, // USB UP
            HttpCameraKind.kMJPGStreamer
    );
    private Camera currentFeed = Camera.INVALID;


    public Vision() {
        climbCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
        climbCamera.setFPS(CAMERA_FPS);
        climbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        intakeCamera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
        intakeCamera.setFPS(CAMERA_FPS);
        intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        videoServer.setSource(intakeCamera);
        currentFeed = Camera.INTAKE;
    }

    public void switchFeed() {
        switch(currentFeed) {
            case INTAKE:
                videoServer.setSource(climbCamera);
                currentFeed = Camera.CLIMB;
                break;
            case CLIMB:
            case INVALID:
            default:
                videoServer.setSource(intakeCamera);
                currentFeed = Camera.INTAKE;
                break;
        }
    }

    public Camera getCurrentFeed() {
        return currentFeed;
    }
}
