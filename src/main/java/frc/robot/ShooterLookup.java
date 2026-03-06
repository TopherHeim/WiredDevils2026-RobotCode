package frc.robot;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShooterLookup {
    private final NavigableMap<Double, Bucket> map = new TreeMap<>();

    public synchronized void addSample(double distance, double velocity) {
        addSample(distance, velocity, 1.0);
    }


    public synchronized void addSample(double distance, double velocity, double weight) {
        if (Double.isNaN(distance) || Double.isNaN(velocity) || weight <= 0 || Double.isInfinite(distance)|| Double.isInfinite(velocity)) {
            return;
        }

        Bucket b = map.get(distance);
        if (b == null) {
            b = new Bucket();
            map.put(distance, b);
        }
        b.add(velocity, weight);
    }

    public synchronized double getRepresentative(double distance) {
        Bucket b = map.get(distance);
        return b == null ? Double.NaN : b.mean();
    }

    public synchronized double getInterpolatedVelocity(double distance) {
        if (map.isEmpty()) {
            return Double.NaN;
        }

        if (map.containsKey(distance)) {
            return map.get(distance).mean();
        }

        Map.Entry<Double, Bucket> low = map.floorEntry(distance);
        Map.Entry<Double, Bucket> high = map.ceilingEntry(distance);

        if (low == null && high == null) {
            return Double.NaN;
        } else if (low == null) {
            return high.getValue().mean();
        } else if (high == null) {
            return low.getValue().mean();
        } else {
            double x0 = low.getKey();
            double y0 = low.getValue().mean();
            double x1 = high.getKey();
            double y1 = high.getValue().mean();
            if (x1 == x0) {
                return 0.5 * (y0 + y1);
            }
            double t = (distance - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }

    public synchronized void initLookup(){
        addSample(0, 0);
    }

    public synchronized double getSmoothedVelocity(double distance, double radius) {
        if (map.isEmpty()) {
            return Double.NaN;
        }

        if (radius <= 0) {
            return getInterpolatedVelocity(distance);
        }

        double lowKey = distance - radius;
        double highKey = distance + radius;

        NavigableMap<Double, Bucket> sub = map.subMap(lowKey, true, highKey, true);
        if (sub.isEmpty()) {
            return getInterpolatedVelocity(distance);
        }

        double weightSum = 0.0;
        double weightedVelSum = 0.0;
        // Gaussian kernel scale: sigma = radius/2 (empirical)
        double sigma = Math.max(1e-6, radius / 2.0);
        double twoSigmaSq = 2 * sigma * sigma;

        for (Map.Entry<Double, Bucket> e : sub.entrySet()) {
            double d = Math.abs(e.getKey() - distance);
            double w = Math.exp(-(d * d) / twoSigmaSq);
            double mean = e.getValue().mean();
            weightedVelSum += w * mean;
            weightSum += w;
        }

        if (weightSum == 0) {
            return getInterpolatedVelocity(distance);
        }
        return weightedVelSum / weightSum;
    }

    public synchronized int getEntryCount() { return map.size(); }

    private static class Bucket {
        private double weightSum = 0.0; // sum of weights
        private double weightedVelSum = 0.0; // sum(w * v)
        private double weightedVelSqSum = 0.0; // sum(w * v^2)

        void add(double velocity, double weight) {
            weightSum += weight;
            weightedVelSum += weight * velocity;
            weightedVelSqSum += weight * velocity * velocity;
        }

        double mean() {
            return weightSum == 0.0 ? Double.NaN : (weightedVelSum / weightSum);
        }

        double variance() {
            if (weightSum == 0.0) return Double.NaN;
            double mean = mean();
            return (weightedVelSqSum / weightSum) - (mean * mean);
        }

        long countApprox() {
            return (long) Math.round(weightSum);
        }
    }
}
