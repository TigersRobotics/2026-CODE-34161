package org.firstinspires.ftc.teamcode.blue;

public enum Alliance {
    RED(30, 37),
    BLUE(38, 45);

    public final int firstTag;
    public final int lastTag;

    Alliance(int firstTag, int lastTag) {
        this.firstTag = firstTag;
        this.lastTag = lastTag;
    }

    public boolean ownsTag(int id) {
        return id >= firstTag && id <= lastTag;
    }

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}
