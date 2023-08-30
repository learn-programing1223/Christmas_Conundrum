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
        if (!bot.clawClosed) {
            bot.claw();
        }
        bot.moveArm();
        bot.straight(4, 1, .2);
        bot.claw();
        bot.straight(4, -1, .2);
        bot.moveArm();
        if (cameraPipeline.whitePixels < 40000) {
            //orange
            bot.turnTo(90, .2);
            bot.straight(16, 1, 0.4);
            bot.turnTo(0, .4);
            bot.straight(16, 1, 0.7);
        }
        else {
            bot.turnTo(-90, .2);
            bot.straight(16, 1, 0.4);
            bot.turnTo(0, .4);
            bot.straight(16, 1, 0.7);
        }
    }
}

