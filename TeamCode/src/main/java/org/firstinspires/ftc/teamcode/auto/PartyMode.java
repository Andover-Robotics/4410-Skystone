package org.firstinspires.ftc.teamcode.auto;

public class PartyMode {
  public static void getRainbowColor(double angle, int[] colors) {
    double x_GB = Math.cos(angle),
        y_R = Math.sin(angle);

    int r = (int) Math.round(y_R * 255);
    int g = (int) Math.round(x_GB >= 0 ? x_GB * 255 : 0);
    int b = (int) Math.round(x_GB < 0 ? -x_GB * 255 : 0);

    colors[0] = r;
    colors[1] = g;
    colors[2] = b;
  }
}
