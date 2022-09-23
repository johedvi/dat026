package bouncing_balls;

public class Vector {
    public double x, y;

    public static double dotProduct(Vector v1, Vector v2) {
        return (v1.x * v2.x + v1.y * v2.y);
    }

    public static double vectorMagnitude(Vector v) {
        return Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2));
    }

    public static double slopeBetweenTwoVectors(Vector v1, Vector v2) {
        return (v1.x- v2.x) / (v1.y - v2.y);
    }

    public static Vector scaleVector(Vector v, double s) {
        return new Vector(v.x * s, v.y * s);
    }

    public static Vector addVectors(Vector v1, Vector v2) {
        return new Vector(v1.x + v2.x, v1.y + v2.y);
    }

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector(Vector v) {
        this.x = v.x;
        this.y = v.y;
    }

    public void scale(double s) {
        x *= s;
        y *= s;
    }
}
