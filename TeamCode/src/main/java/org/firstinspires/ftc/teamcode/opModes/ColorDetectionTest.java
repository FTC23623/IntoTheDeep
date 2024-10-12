package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "ColorDetectionTest")
public class ColorDetectionTest extends LinearOpMode {

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode () {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                switch (blocks[0].id) {
                    case 1:
                        // blue
                        pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                        break;
                    case 2:
                        // red
                        pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                        break;
                    case 3:
                        // yellow
                        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                        break;
                    default:
                        pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY;
                        break;
                }
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY;
            }
            blinkinLedDriver.setPattern(pattern);
            sleep(20);
        }
    }
}
