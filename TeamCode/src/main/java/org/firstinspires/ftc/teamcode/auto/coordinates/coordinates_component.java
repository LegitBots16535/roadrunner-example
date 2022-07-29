package org.firstinspires.ftc.teamcode.auto.coordinates;
import java.util.*;
import java.util.HashMap;

public class coordinates_component {

    public static HashMap<String, ArrayList<Integer>> map=new HashMap<>();
    public static int x = 5;

    public static void putValues(String key, ArrayList values) {
        map.put(key, values);

    }

    public static int getXValue(String key) {
        ArrayList<Integer> x = map.get(key);

        return x.get(0);

    }

    public static int getYValue(String key) {
        ArrayList<Integer> y = map.get(key);

        return y.get(1);

    }






}


