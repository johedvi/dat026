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

    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[4];
        balls[0] = new Ball(width / 6, height * .8, 1, -1, 0.3, 2);
        balls[1] = new Ball(2 * width / 6, height * .5, 0, 0, 0.4, 3);
        balls[2] = new Ball(3 * width / 6, height * .5, 0, 0, 0.2, 1);
        balls[3] = new Ball(width / 6, height * .3, 0, 0, 0.5, 4);
    }


    void step(double deltaT) {

        for (int i = 0; i < balls.length; i++) {
            
            Ball ball_0 = balls[i];

            for(int j = i+1; j < balls.length; j++) {

                Ball ball_1 = balls[j];

                if (circlesIsIntersecting(ball_0, ball_1)) {
                    // Calculate angle in radians between the x-axis and the line between the balls centres (l)
                    double slopeBetweenBalls = Vector.slopeBetweenTwoVectors(ball_0.position, ball_1.position);
                    double slopeAxisX = 0;
                    double tanBetweenLines = acuteAngleBetweenLines(slopeBetweenBalls, slopeAxisX);
                    double radianAngleBetweenLines = Math.atan(tanBetweenLines);
    
                    // Convert cartesian-vectors to polar-vectors
                    Vector polarVector0 = new Vector(Vector.vectorMagnitude(ball_0.velocity), ball_0.velocity.getDirectionInRadians());
                    Vector polarVector1 = new Vector(Vector.vectorMagnitude(ball_1.velocity), ball_1.velocity.getDirectionInRadians());
    
                    // Project the velocity vectors to the new system
                    Vector projectedVector0 = new Vector(polarVector0.x * Math.cos(polarVector0.y - radianAngleBetweenLines),
                            polarVector0.x * Math.sin(polarVector0.y - radianAngleBetweenLines));
    
                    Vector projectedVector1 = new Vector(polarVector1.x * Math.cos(polarVector1.y - radianAngleBetweenLines),
                            polarVector1.x * Math.sin(polarVector1.y - radianAngleBetweenLines));
    
                    double initialProjectedVelocityX0 = projectedVector0.x;
                    double initialProjectedVelocityX1 = projectedVector1.x;
    
                    double v1fxr = ((ball_0.mass - ball_1.mass) * initialProjectedVelocityX0 +
                            (ball_1.mass + ball_1.mass) * initialProjectedVelocityX1) /
                            (ball_0.mass + ball_1.mass);
    
                    double v2fxr = ((ball_0.mass + ball_0.mass) * initialProjectedVelocityX0 +
                            (ball_1.mass - ball_0.mass) * initialProjectedVelocityX1) /
                            (ball_0.mass + ball_1.mass);
    
                    Vector projectedVectorAfterCollision0 = new Vector(
                            v1fxr, projectedVector0.y);
    
                    Vector projectedVectorAfterCollision1 = new Vector(v2fxr, projectedVector1.y);
    
                    // Fix ball overlapping
                    double overlap = ball_0.radius + ball_1.radius - distanceBetween(ball_0, ball_1);
                    ball_0.position.x += overlap / 2 * Math.signum(projectedVectorAfterCollision0.x);
                    ball_1.position.x += overlap / 2 * Math.signum(projectedVectorAfterCollision1.x);
    
                    Vector stdCartesianAfterCollision0 = new Vector(
                            Math.cos(radianAngleBetweenLines) * projectedVectorAfterCollision0.x +
                                    Math.cos(radianAngleBetweenLines + Math.PI / 2.0) * projectedVectorAfterCollision0.y,
                            Math.sin(radianAngleBetweenLines) * projectedVectorAfterCollision0.x +
                                    Math.sin(radianAngleBetweenLines + Math.PI / 2.0) * projectedVectorAfterCollision0.y);
    
                    Vector stdCartesianAfterCollision1 = new Vector(
                            Math.cos(radianAngleBetweenLines) * projectedVectorAfterCollision1.x +
                                    Math.cos(radianAngleBetweenLines + Math.PI / 2.0) * projectedVectorAfterCollision1.y,
                            Math.sin(radianAngleBetweenLines) * projectedVectorAfterCollision1.x +
                                    Math.sin(radianAngleBetweenLines + Math.PI / 2.0) * projectedVectorAfterCollision1.y);
    
                    ball_0.velocity = stdCartesianAfterCollision0;
                    ball_1.velocity = stdCartesianAfterCollision1;
                }
                
            }
            

            if (ball_0.position.x < ball_0.radius || ball_0.position.x > areaWidth - ball_0.radius) {
                ball_0.velocity.x *= -1; // change direction of ball
            }
            if (ball_0.position.y < ball_0.radius || ball_0.position.y > areaHeight - ball_0.radius) {
                ball_0.velocity.y *= -1;
            }

            moveBalls(deltaT, ball_0);
        
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

        // Adjust position so no sex with floor
        if(b.position.y < b.radius)
            b.position.y = b.radius;
        else if(b.position.y > areaHeight - b.radius)
            b.position.y = areaHeight - b.radius;

        // Gravity
        b.velocity.y -= 9.82 * deltaT;

        // Moves ball
        b.position.x += deltaT * b.velocity.x;
        b.position.y += deltaT * b.velocity.y;

    }

    double calculateNewVelocityX(double mass1, double v1, double mass2, double v2) {
        return ((mass1 - mass2) / (mass1 + mass2)) * v1
                +
                ((2 * mass2) / (mass1 + mass2)) * v2;
    }

    boolean circlesIsIntersecting(Ball b1, Ball b2) {
        // Pythagoras
        double distanceBetweenCircles = Math.sqrt(Math.pow(b1.position.x - b2.position.x, 2) + Math.pow(b1.position.y - b2.position.y, 2));
        double totalRadius = b1.radius + b2.radius;
        return distanceBetweenCircles < totalRadius;
    }

    double acuteAngleBetweenLines(double slope1, double slope2) {
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
