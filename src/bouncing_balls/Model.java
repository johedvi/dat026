package bouncing_balls;

import java.util.Arrays;

/**
 * The physics model.
 * <p>
 * This class is where you should implement your bouncing balls model.
 * <p>
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 *
 * @author Simon Robillard
 */
class Model {

    double areaWidth, areaHeight;

    Ball[] balls;

    double totalRadius;



    boolean ballsIsFrozen = false;
    boolean ballsIsInGround = true;

    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[2];
        balls[0] = new Ball(width / 3, height * 0.5, 1, 0.25, 0.2, 3);
        balls[1] = new Ball(2 * width / 3, height * 0.5, -1.25, -0.5, 0.3, 2);

        totalRadius = balls[0].radius + balls[1].radius;
    }


    void step(double deltaT) {
        // TODO this method implements one step of simulation with a step deltaT

        for (Ball b : balls) {
            // detect collision with the border

            if (circlesIsIntersecting()) {

                collision(balls[0],balls[1]);

            }

            if (b.position.x < b.radius || b.position.x > areaWidth - b.radius) {

                b.velocity.x *= -1; // change direction of ball

            }
            if (b.position.y < b.radius || b.position.y > areaHeight - b.radius) {

                b.velocity.y *= -1;

            } else {
                ballsIsInGround = false;
            }

            if(!ballsIsInGround) {
               b.velocity.y -= 0.2;
            }

            moveBalls(deltaT, b);



        }


        // Handle collisions in between circles
    }




    private void moveBalls(double deltaT, Ball b) {
        // compute new position according to the speed of the ball
        b.position.x += deltaT * b.velocity.x;
        b.position.y += deltaT * b.velocity.y;
    }

    boolean circlesIsIntersecting() {
        // Pythagoras
        double distanceBetweenCircles = Math.sqrt(Math.pow(balls[0].position.x - balls[1].position.x, 2) + Math.pow(balls[0].position.y - balls[1].position.y, 2));
        return distanceBetweenCircles < totalRadius;
    }




    // #https://www.youtube.com/watch?v=guWIF87CmBg lemao

    void collision (Ball b1, Ball b2) {

        double vx1 = b1.velocity.x;
        double vy1 = b1.velocity.y;
        double vx2 = b2.velocity.x;
        double vy2 = b2.velocity.y;

        Vector v1 = new Vector(b1.velocity.x, b1.velocity.y);
        Vector v2 = new Vector(b2.velocity.x, b2.velocity.y);


        double dx = Math.abs(b1.position.x - b2.position.x);
        double dy = Math.abs(b1.position.y - b2.position.y);

        double contactAngle = Math.atan(dy/dx);
        double [][] rotation = generateRotationMatrix(contactAngle);
        double [][] rotationInverse = generateInverseRotationMatrix(contactAngle);


        v1 = new Vector(rotation[0][0]*v1.x + rotation[0][1]*v1.y,rotation[1][0]*v1.x + rotation[1][1]*v1.y);
        v2 = new Vector(rotation[0][0]*v2.x + rotation[0][1]*v2.y,rotation[1][0]*v2.x + rotation[1][1]*v2.y);


        v1.x = (vx1 * (b1.mass - b2.mass) + 2 * b2.mass * vx2) / (b1.mass + b2.mass);
        v2.x = (vx2 * (b2.mass - b1.mass) + 2 * b1.mass * vx1) / (b1.mass + b2.mass);
        v1.y = (vy1 * (b1.mass - b2.mass) + 2 * b2.mass * vy2) / (b1.mass + b2.mass);
        v2.y = (vy2 * (b2.mass - b1.mass) + 2 * b1.mass * vy1) / (b1.mass + b2.mass);


        v1 = new Vector(rotationInverse[0][0]*v1.x + rotationInverse[0][1]*v1.y,rotationInverse[1][0]*v1.x + rotationInverse[1][1]*v1.y);
        v2 = new Vector(rotationInverse[0][0]*v2.x + rotationInverse[0][1]*v2.y,rotationInverse[1][0]*v2.x + rotationInverse[1][1]*v2.y);


        b1.velocity.x = v1.x;
        b1.velocity.y = v1.y;
        b2.velocity.x = v2.x;
        b2.velocity.y = v2.y;
    }




    double[][] generateRotationMatrix(double radianAngle) {
        double[][] rotationMatrix = new double[2][2];
        rotationMatrix[0][0] = Math.cos(radianAngle);
        rotationMatrix[0][1] = -Math.sin(radianAngle);
        rotationMatrix[1][0] = Math.sin(radianAngle);
        rotationMatrix[1][1] = Math.cos(radianAngle);
        return rotationMatrix;
    }

    double[][] generateInverseRotationMatrix(double radianAngle) {
        double[][] rotationMatrix = new double[2][2];
        rotationMatrix[0][0] = Math.cos(radianAngle);
        rotationMatrix[0][1] = Math.sin(radianAngle);
        rotationMatrix[1][0] = -Math.sin(radianAngle);
        rotationMatrix[1][1] = Math.cos(radianAngle);
        return rotationMatrix;
    }


    /**
     * Simple inner class describing balls.
     */
    class Ball {
        Vector position;
        Vector velocity;
        double radius;
        double mass;

        Ball(double x, double y, double vx, double vy, double r, double m) {
            position = new Vector(x, y);
            velocity = new Vector(vx, vy);
            radius = r;
            mass = m;
        }
    }

}
