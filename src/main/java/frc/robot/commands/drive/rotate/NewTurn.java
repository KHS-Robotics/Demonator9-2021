import edu.wpi.first.wpilibj.geometry.Translation2d;

public class NewTurn extends CommandBase{
    private Translation2d centerPoint;
    public NewTurn(Translation2d centerPoint) {
        this.centerPoint = centerPoint;
    }

    public void pizza(Translation2d modulePos, Pose2d moduleRot, double radius) {
        
    }

    public boolean testPoint(double x, double y, double radius, double h, double k) {
        return (x-h) * (x-h) + (y-k) * (y-k) == radius * radius;
    }

    public Translation2d furthestFromCenter(Translation2d mod1, Translation2d mod2, Translation2d mod3, Translation2d mod4) {
        double mod1Distance = distanceFromCenter(mod1);
        double mod2Distance = distanceFromCenter(mod2);
        double mod3Distance = distanceFromCenter(mod3);
        double mod4Distance = distanceFromCenter(mod4);

        Translation2d largest = mod1;
        double largestDistance = mod1Distance;
        if (mod2Distance > largestDistance) {
            largest = mod2;
            largestDistance = mod2Distance;
        }
        if (mod3Distance > largestDistance) {
            largest = mod3;
            largestDistance = mod3Distance;
        }
        if (mod4Distance > largestDistance) {
            largest = mod4;
            largestDistance = mod4Distance;
        }

        return largest;
    }

    private double distanceFromCenter(Translation2d point) {
        double distance = Math.sqrt(Math.pow(point.getX() - centerPoint.getX(), 2) + Math.pow(point.getY() - centerPoint.getY(), 2);
        return distance;
    }
}