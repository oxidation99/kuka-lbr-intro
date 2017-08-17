package me.nicholasnadeau.robot.kukalbr.phd.state;

public enum ESMState {
    NORMAL("1"),
    HAND_GUIDING("2");

    private String value;

    ESMState(String value) {
        this.value = value;
    }

    public String getValue() {
        return value;
    }
}
