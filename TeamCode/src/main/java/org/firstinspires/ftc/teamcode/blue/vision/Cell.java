package org.firstinspires.ftc.teamcode.blue.vision;

public class Cell {
    public final int firstTag;
    public final double tx;
    public final double ty;
    public final int tags;

    public Cell(int firstTag, double tx, double ty, int tags) {
        this.firstTag = firstTag;
        this.tx = tx;
        this.ty = ty;
        this.tags = tags;
    }

    @Override
    public String toString() {
        return String.format("%d-%d  tx %.1f  ty %.1f  (%d tags)", firstTag, firstTag + 3, tx, ty, tags);
    }
}
