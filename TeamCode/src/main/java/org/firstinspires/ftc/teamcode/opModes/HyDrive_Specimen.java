package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.types.ElementTypes;

@TeleOp(name="HyDrive_Specimen")
public class HyDrive_Specimen extends HyDrive {
    public HyDrive_Specimen() {
        mStartElementType = ElementTypes.Specimen;
    }
}
