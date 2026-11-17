package org.firstinspires.ftc.teamcode.sensing;

import static org.firstinspires.ftc.teamcode.constants.SensingConstants.TIP_DECIDE_LEVEL;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.TIP_VOTE_GAIN;

/**
 * Remembers which way each hive is tipped.
 * <p>
 * A hive is a see-saw: exactly one of its two cells is up. Every camera frame that clearly shows a
 * cell up or down is one vote. Votes are smoothed, so a single bad frame cannot flip the answer,
 * and the answer only changes once the votes are solidly on the other side. Because the hive stays
 * put until it is tipped again, the last answer is kept while the camera cannot see it.
 * <p>
 * No FTC SDK in here, and nothing is allocated after construction.
 */
public final class HiveTracker {
    public enum State {
        /** The camera has not had a clear look at this hive yet. */
        UNKNOWN,
        /** The cell on the audience side of the field is the one pointing up. */
        AUDIENCE_CELL_UP,
        /** The cell on the far (scoring-table) side of the field is the one pointing up. */
        SCORING_CELL_UP
    }

    public static final int RED_HIVE = 0;
    public static final int BLUE_HIVE = 1;

    // +1 = sure the audience cell is up, -1 = sure the scoring cell is up, 0 = no idea
    private final double[] evidence = new double[2];
    private final State[] state = { State.UNKNOWN, State.UNKNOWN };
    private final long[] lastSeenNs = { Long.MIN_VALUE, Long.MIN_VALUE };
    private final long[] lastTipNs = { Long.MIN_VALUE, Long.MIN_VALUE };
    private final int[] tipCount = new int[2];

    /**
     * @param redStart  +1 if the red hive starts with its audience cell up, -1 scoring cell, 0 unknown
     * @param blueStart same for the blue hive
     */
    public void reset(int redStart, int blueStart) {
        setStart(RED_HIVE, redStart);
        setStart(BLUE_HIVE, blueStart);
    }

    private void setStart(int hive, int start) {
        evidence[hive] = start > 0 ? 1 : start < 0 ? -1 : 0;
        state[hive] = start > 0 ? State.AUDIENCE_CELL_UP : start < 0 ? State.SCORING_CELL_UP : State.UNKNOWN;
        lastSeenNs[hive] = Long.MIN_VALUE;
        lastTipNs[hive] = Long.MIN_VALUE;
        tipCount[hive] = 0;
    }

    /**
     * One camera frame's opinion.
     *
     * @param audienceCellUp true if the frame shows the audience cell up (or the scoring cell down)
     */
    public void vote(int hive, boolean audienceCellUp, long nowNs) {
        double e = evidence[hive];
        e += TIP_VOTE_GAIN * ((audienceCellUp ? 1.0 : -1.0) - e);
        evidence[hive] = e;
        lastSeenNs[hive] = nowNs;

        State old = state[hive];
        State now = e > TIP_DECIDE_LEVEL ? State.AUDIENCE_CELL_UP
                : e < -TIP_DECIDE_LEVEL ? State.SCORING_CELL_UP
                : old; // not sure enough to change the answer

        if (now != old) {
            state[hive] = now;
            if (old != State.UNKNOWN) {
                tipCount[hive]++;
                lastTipNs[hive] = nowNs;
            }
        }
    }

    public State getState(int hive) { return state[hive]; }

    /** 0 = no idea, 1 = certain. */
    public double getConfidence(int hive) { return Math.abs(evidence[hive]); }

    /** How many times this hive has been seen to tip since reset. */
    public int getTipCount(int hive) { return tipCount[hive]; }

    /** Milliseconds since this hive last tipped, or infinity if it has not. */
    public double msSinceTip(int hive, long nowNs) {
        return lastTipNs[hive] == Long.MIN_VALUE ? Double.POSITIVE_INFINITY : (nowNs - lastTipNs[hive]) * 1e-6;
    }

    /** Milliseconds since the camera last had a clear look at this hive, or infinity if never. */
    public double msSinceSeen(int hive, long nowNs) {
        return lastSeenNs[hive] == Long.MIN_VALUE ? Double.POSITIVE_INFINITY : (nowNs - lastSeenNs[hive]) * 1e-6;
    }
}
