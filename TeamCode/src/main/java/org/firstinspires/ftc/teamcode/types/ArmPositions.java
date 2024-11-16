package org.firstinspires.ftc.teamcode.types;

/**
 * - Arm positions
 *   - Home
 *     - Lift at 0 ticks / -15 degrees
 *     - Slides at 0 ticks
 *     - Wrist at ?? degrees
 *   - Manual floor pickup
 *     - Can we enforce that we can only start this from home?
 *     - Use joystick to extend slides. (Set power to motor based on joystick position)
 *     - Use trigonometry to lift the arm while extending
 *     - Button press to pick up once positioned over sample?
 *   - Floor pickup (designed for auto)
 *     - Slides at minimal extension to reach floor
 *     - Lift at ?? degrees
 *     - Wrist at ?? degrees
 *   - Specimen pickup
 *     - Lift at ?? degrees
 *     - Extension 0 inches
 *     - Wrist at 0 degrees
 *   - Specimen lower chamber
 *     - Lift at ?? degrees
 *     - Extension at ?? inches
 *     - Wrist at ?? degrees
 *     - How do we score? Keep wrist at an angle and pull the extension in manually?
 *   - Specimen upper chamber
 *     - Lift at ?? degrees
 *     - Extension at ?? inches
 *     - Wrist at ?? degrees
 *     - How do we score? Keep wrist at an angle and pull the extension in manually?
 *   - Sample lower basket
 *     - Lift at ?? degrees
 *     - Extension at ?? inches
 *     - Wrist at ?? degrees
 *     - Score by reversing intake
 *   - Sample upper basket
 *     - Lift at ?? degrees
 *     - Extension at ?? inches
 *     - Wrist at ?? degrees
 *     - Score by reversing intake
 *
 */
public enum ArmPositions {
    Pos0Home,
    Pos1ManualPickup,
    Pos2FloorPickup,
    Pos3SpecimenPickup,
    Pos4SpecimenLowerChamber,
    Pos5SpecimenUpperChamber,
    Pos6SampleLowerBasket,
    Pos7SampleUpperBasket,
    Pos8Carry,
    Pos9Ascent1,
    Pos10Temp,
    Pos11Turtle
}
