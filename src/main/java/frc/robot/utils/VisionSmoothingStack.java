package frc.robot.utils;

import java.util.LinkedList;

import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * <h3>VisionSmoothingStack</h3>
 * 
 * Smoothes the vision results from PhotonVision. This gives us a better target
 * to use, otherwise we would be jittering all over the place
 */
public class VisionSmoothingStack {
    /**
     * The size of the vision smoothing stack
     */
    private final int stackSize;

    /**
     * The list that will hold all of the vision results
     */
    private final LinkedList<PhotonTrackedTarget> results;

    /**
     * <h3>VisionSmoothingStack</h3>
     * 
     * Initializes a new {@link frc.robot.utilities.VisionSmoothingStack
     * VisionSmoothingStack} with the passed stack size
     * 
     * @param size the size of the stack
     */
    public VisionSmoothingStack(int size) {
        stackSize = size;
        results = new LinkedList<>();
    }

    /**
     * <h3>addItem</h3>
     * 
     * Add the passed result to the list and pop the oldest one if the list is
     * greater than the set size.
     * 
     * @param result the {@link org.photonvision.targeting.PhotonTrackedTarget
     *               PhotonTrackedTarget} to add to the list
     */
    public void addItem(PhotonTrackedTarget result) {
        if (results.size() >= stackSize) {
            results.removeFirst();
        }
        results.add(result);
    }

    /**
     * <h3>getAveragePitch</h3>
     * 
     * Average the pitch of all the results
     * 
     * @return the average pitch of all the results
     */
    public double getAveragePitch() {
        double total = 0;
        for (PhotonTrackedTarget target : results) {
            total += target.getPitch();
        }
        return (total / results.size());
    }

    /**
     * <h3>getAverageYaw</h3>
     * 
     * Average the yaw of all the results
     * 
     * @return the average yaw of all the results
     */
    public double getAverageYaw() {
        double total = 0;
        for (PhotonTrackedTarget target : results) {
            total += target.getYaw();
        }
        return (total / results.size());
    }
}