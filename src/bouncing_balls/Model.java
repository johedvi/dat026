package bouncing_balls;

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

    double gravity = 9.82;
    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[2];
        balls[0] = new Ball(width / 3, height * .5, 0, 0, 0.3, 1);
        balls[1] = new Ball(2 * width / 3, height * .5, 0, 0, 0.3, 1);

        totalRadius = balls[0].radius + balls[1].radius;
    }


    void step(double deltaT) {
        // If mass is not assigned here it equals to 0

        Ball ball_0 = balls[0];
        Ball ball_1 = balls[1];


        for (Ball b : balls) {
            if (circlesIsIntersecting()) {
                // Calculate collision angle
                double horizontalDelta = balls[0].position.x - balls[1].position.x;
                double verticalDelta = balls[0].position.y - balls[1].position.y;
                double collisionAngle = horizontalDelta == 0 ? (Math.PI / 2.0) : Math.atan(verticalDelta / horizontalDelta);

                // Convert cartesian-vectors to polar-vectors
                Vector polarVector0 = new Vector(Vector.vectorMagnitude(ball_0.velocity),
                        ball_0.velocity.getDirectionInRadians());
                Vector polarVector1 = new Vector(Vector.vectorMagnitude(ball_1.velocity),
                        ball_1.velocity.getDirectionInRadians());

                // Project the velocity vectors to the new system
                Vector projectedVector0 = new Vector(polarVector0.x * Math.cos(polarVector0.y - collisionAngle),
                        polarVector0.x * Math.sin(polarVector0.y - collisionAngle));
                Vector projectedVector1 = new Vector(polarVector1.x * Math.cos(polarVector1.y - collisionAngle),
                        polarVector1.x * Math.sin(polarVector1.y - collisionAngle));

                // Cache the initial projected x-component of the velocity vector
                double initialProjectedVelocityX0 = projectedVector0.x;
                double initialProjectedVelocityX1 = projectedVector1.x;

                // Calculate the resulting velocity in the projected space
                double projectedVelocityPostCollisionX0 = ((ball_0.mass - ball_1.mass) * initialProjectedVelocityX0 +
                        (ball_1.mass + ball_1.mass) * initialProjectedVelocityX1) /
                        (ball_0.mass + ball_1.mass);
                double projectedVelocityPostCollisionX1 = ((ball_0.mass + ball_0.mass) * initialProjectedVelocityX0 +
                        (ball_1.mass - ball_0.mass) * initialProjectedVelocityX1) /
                        (ball_0.mass + ball_1.mass);

                // Construct a new vector with the new x-component
                Vector projectedVectorAfterCollision0 = new Vector(
                        projectedVelocityPostCollisionX0, projectedVector0.y);
                Vector projectedVectorAfterCollision1 = new Vector(projectedVelocityPostCollisionX1, projectedVector1.y);

                // Fix ball overlapping
                double overlap = ball_0.radius + ball_1.radius - distanceBetween(ball_0, ball_1);
                ball_0.position.x += overlap / 2 * Math.signum(projectedVectorAfterCollision0.x);
                ball_1.position.x += overlap / 2 * Math.signum(projectedVectorAfterCollision1.x);

                // Convert back to standard cartesian 2D space
                Vector stdCartesianAfterCollision0 = new Vector(
                        Math.cos(collisionAngle) * projectedVectorAfterCollision0.x +
                                Math.cos(collisionAngle + Math.PI / 2.0) * projectedVectorAfterCollision0.y,
                        Math.sin(collisionAngle) * projectedVectorAfterCollision0.x +
                                Math.sin(collisionAngle + Math.PI / 2.0) * projectedVectorAfterCollision0.y);
                Vector stdCartesianAfterCollision1 = new Vector(
                        Math.cos(collisionAngle) * projectedVectorAfterCollision1.x +
                                Math.cos(collisionAngle + Math.PI / 2.0) * projectedVectorAfterCollision1.y,
                        Math.sin(collisionAngle) * projectedVectorAfterCollision1.x +
                                Math.sin(collisionAngle + Math.PI / 2.0) * projectedVectorAfterCollision1.y);

                // Assign the new velocity
                ball_0.velocity = stdCartesianAfterCollision0;
                ball_1.velocity = stdCartesianAfterCollision1;
            }

            if (b.position.x < b.radius || b.position.x > areaWidth - b.radius) {
                b.velocity.x *= -1; // change direction of ball
            }
            if (b.position.y < b.radius || b.position.y > areaHeight - b.radius) {
                b.velocity.y *= -1;
            }

            moveBalls(deltaT, b);


        }
    }

    private double distanceBetween(Ball b1, Ball b2) {
        double deltaY = b2.position.y - b1.position.y;
        double deltaX = b2.position.x - b1.position.x;
        return Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
    }

    private void moveBalls(double deltaT, Ball b) {
        if(b.position.x < b.radius)
            b.position.x = b.radius;
        else if(b.position.x > areaWidth - b.radius)
            b.position.x = areaWidth - b.radius;

        if(b.position.y < b.radius)
            b.position.y = b.radius;
        else if(b.position.y > areaHeight - b.radius)
            b.position.y = areaHeight - b.radius;

        // Gravity
        b.velocity.y -= gravity * deltaT;

        // Moves ball
        b.position.x += deltaT * b.velocity.x;
        b.position.y += deltaT * b.velocity.y;

    }

    double calculateNewVelocityX(double mass1, double v1, double mass2, double v2) {
        return ((mass1 - mass2) / (mass1 + mass2)) * v1
                +
                ((2 * mass2) / (mass1 + mass2)) * v2;
    }

    boolean circlesIsIntersecting() {
        // Pythagoras
        double distanceBetweenCircles = Math.sqrt(Math.pow(balls[0].position.x - balls[1].position.x, 2) + Math.pow(balls[0].position.y - balls[1].position.y, 2));
        return distanceBetweenCircles < totalRadius;
    }

    double collisionAngleWithRegardsToAxisX(double slope1, double slope2) {
        return (slope1 - slope2) / (1 + (slope1 * slope2));
    }

    double[][] generateRotationMatrixCounterClockwise(double radianAngle) {
        double[][] rotationMatrix = new double[2][2];
        rotationMatrix[0][0] = Math.cos(radianAngle);
        rotationMatrix[0][1] = -Math.sin(radianAngle);
        rotationMatrix[1][0] = Math.sin(radianAngle);
        rotationMatrix[1][1] = Math.cos(radianAngle);
        return rotationMatrix;
    }

    double[][] generateRotationMatrixClockwise(double radianAngle) {
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
        Vector oldPosition = new Vector(0, 0);
        Vector velocity;
        double radius;
        double mass;

        Ball(double x, double y, double vx, double vy, double r, double m) {
            this.position = new Vector(x, y);
            this.velocity = new Vector(vx, vy);
            this.radius = r;
            this.mass = m;
        }
    }

}
