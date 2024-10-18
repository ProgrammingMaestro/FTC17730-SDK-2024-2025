package vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class CameraOpMode extends LinearOpMode {
    OpenCvCamera camera;
    ColorDetectionPipeline colorPipeline;

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()){
        }
    }
}
