package org.firstinspires.ftc.teamcode.datalogger;

public class ArmDatalogger {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;
    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField battVoltage = new Datalogger.GenericField("Batt V");
    public Datalogger.GenericField liftTarget = new Datalogger.GenericField("Lft Tgt");
    public Datalogger.GenericField liftPosition = new Datalogger.GenericField("Lft Pos");
    public Datalogger.GenericField liftCurrent = new Datalogger.GenericField("Lft Curr");
    public Datalogger.GenericField liftPower = new Datalogger.GenericField("Lft Pwr");
    public Datalogger.GenericField extendTarget = new Datalogger.GenericField("Ext Tgt");
    public Datalogger.GenericField extendPosition = new Datalogger.GenericField("Ext Pos");
    public Datalogger.GenericField extendCurrent = new Datalogger.GenericField("Ext Curr");
    public Datalogger.GenericField extendPower = new Datalogger.GenericField("Ext Pwr");
    public Datalogger.GenericField state = new Datalogger.GenericField("State");
    public Datalogger.GenericField action = new Datalogger.GenericField("Action");

    public ArmDatalogger(String name)
    {
        // append the date to the filename
        java.util.Date now = new java.util.Date(System.currentTimeMillis());
        name += "-" + now;
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        battVoltage,
                        liftTarget,
                        liftPosition,
                        liftCurrent,
                        liftPower,
                        extendTarget,
                        extendPosition,
                        extendCurrent,
                        extendPower,
                        state,
                        action
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}
