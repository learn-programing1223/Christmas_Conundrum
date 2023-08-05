package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Drive extends LinearOpMode {
    public void runOpMode() {
        Robot bot = new Robot(hardwareMap, this);

        while(opModeIsActive());

        double lx = gamepad1.left_stick_x * -1;
        double rx = gamepad1.right_stick_x;
        double ly = gamepad1.left_stick_y;

        double lf = lx+ly+rx;
        double lb = lx+ly-rx;
        double rf = lx-ly+rx;
        double rb = lx-ly-rx;
    }
}