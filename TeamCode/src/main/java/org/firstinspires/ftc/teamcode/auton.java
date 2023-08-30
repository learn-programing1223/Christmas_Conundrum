package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton", group="auto")
public class auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);
        bot.initOpenCV();
        FtcDashboard.getInstance().startCameraStream(bot.webcam, 60);

        waitForStart();
        telemetry.addData("white Pixels: ", cameraPipeline.whitePixels);
        telemetry.update();
        sleep(5000);
    }
}

