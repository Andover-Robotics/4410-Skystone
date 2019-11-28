package org.firstinspires.ftc.teamcode.auto;

public enum StonePosition {
  LEFT(0),
  CENTER(1),
  RIGHT(2);

  public final int offsetLeft;

  StonePosition(int posLeft) {
    this.offsetLeft = posLeft;
  }

  public int offsetRight() {
    return 2 - offsetLeft;
  }
}
