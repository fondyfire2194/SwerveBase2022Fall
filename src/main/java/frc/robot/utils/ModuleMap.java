package frc.robot.utils;

import java.util.*;

import frc.robot.Constants.DriveConstants.ModulePosition;

/**
 * Contains functions to convert {@link Map}s with {@link ModulePosition} keys to and from arrays so
 * that it's easier to use WPILib swerve functions.
 */
public class ModuleMap {

  /**
   * Creates a {@code Map} with {@link ModulePosition} keys from multiple values, in the order
   * specified in the {@link ModulePosition} enum.
   *
   * <p>For processing the output of a WPILib swerve function which returns an array.
   *
   * @param values Must have at least as many elements as {@link ModulePosition} has entries. Any
   *     entries after will be ignored.
   */
  @SafeVarargs
  public static <V> Map<ModulePosition, V> of(V... values) {
    Map<ModulePosition, V> map = new HashMap<>();
    for (int i = 0; i < ModulePosition.values().length; i++) {
      map.put(ModulePosition.values()[i], values[i]);
    }
    return map;
  }

  /**
   * Returns the values from a map as a {@link List} in the same order as in the {@link
   * ModulePosition} enum.
   *
   * <p>You can use this in a for/in loop without needing to supply an empty array like in {@link
   * #orderedValues(Map, Object[]) orderedValues}.
   */
  public static <V> List<V> orderedValuesList(Map<ModulePosition, V> map) {
    ArrayList<V> list = new ArrayList<>();
    for (ModulePosition i : ModulePosition.values()) {
      list.add(map.get(i));
    }
    return list;
  }

  /**
   * Returns the values from the map as an {@code Array} in the same order as in the {@link
   * ModulePosition} enum.
   *
   * <p>Useful when a WPILib swerve function requires an array as input.
   *
   * @param array An array of the class to output an array of, e.g. {@code
   *     moduleTranslations.valuesArray(new Translation2d[0])}. Required because Java can't make an
   *     array of generics.
   */
  public static <V> V[] orderedValues(Map<ModulePosition, V> map, V[] array) {
    return orderedValuesList(map).toArray(array);
  }
}
