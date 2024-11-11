package team696.frc.lib.Datatypes;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;
import team696.frc.lib.Util;

public class InterpolatingTable<T extends Interpolatable<T>> {
    public final TreeMap<Double, T> map = new TreeMap<Double, T>();

    @SafeVarargs
    public InterpolatingTable(Map.Entry<Double, T>... entries) {
        for (Map.Entry<Double, T> entry : entries) {
            map.putIfAbsent(entry.getKey(), entry.getValue());
        }
    }
    
    public T getValue(double key) {
        double index = Util.clamp(key, map.firstKey() + 0.0000000001, map.lastKey() - 0.0000000001);
        Map.Entry<Double, T> lower = map.floorEntry(index);
        Map.Entry<Double, T> higher = map.ceilingEntry(index);

        double t = (index - lower.getKey()) / (higher.getKey() - lower.getKey());

        return lower.getValue().interpolate(higher.getValue(), t);
    }

}
