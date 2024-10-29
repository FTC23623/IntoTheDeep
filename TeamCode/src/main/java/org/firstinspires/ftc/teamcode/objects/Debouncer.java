package org.firstinspires.ftc.teamcode.objects;

/**
 * Debounce an input
 */
public class Debouncer {
    // how many times to see the input high before debounced
    private final int mActionCount;
    // number of times signal has been high
    private int mConsecutiveCount;
    // optional to see false until the condition goes away again
    private boolean mUsed;

    /**
     * Construct a debouncer
     * @param trueAtCount: number of times input should be true before returning true
     */
    public Debouncer(int trueAtCount) {
        mActionCount = trueAtCount;
        mConsecutiveCount = 0;
        mUsed = false;
    }

    /**
     * Call with the current state of the input to debounce
     * @param state: the current state of the input
     */
    public void In(boolean state) {
        if (state) {
            // increment without overflow
            if (mConsecutiveCount < mActionCount) {
                ++mConsecutiveCount;
            }
        } else {
            // reset count
            mConsecutiveCount = 0;
            // reset used to prepare for the next event
            mUsed = false;
        }
    }

    /**
     * Returns debounced input
     * @return true when debounced input is true and has not been consumed, false otherwise
     */
    public boolean Out() {
        return !mUsed && (mConsecutiveCount >= mActionCount);
    }

    /**
     * Call to force output to false until the input condition goes false again
     */
    public void Used() {
        mUsed = true;
    }
}
