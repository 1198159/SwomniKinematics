import com.b1nary.swomni.ModuleState;
import com.b1nary.swomni.SwomniKinematics;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class SwomniTest {
    public static final double WHEELBASE = 10, TRACKWIDTH = 10;

    @Test
    public void testSwerveKinematics(){
        Vector2D translation = new Vector2D(1,0);
        double rotation = 0.05;
        Vector2D[] states = ModuleState.toVectors(SwomniKinematics.swerveKinematics(translation,rotation,WHEELBASE,TRACKWIDTH));
        System.out.println(Arrays.toString(states));
        //assert Arrays.toString(states).equals("[{0.75; 0.25}, {0.75; -0.25}, {1.25; -0.25}, {1.25; 0.25}]");

    }
    @Test
    public void testSwomniKinematics(){
        Vector2D translation = new Vector2D(1,0);
        double rotation = 0.05;
        double cvt = 1.5;
        Vector2D[] states = ModuleState.toVectors(SwomniKinematics.kinematics(translation,rotation,cvt, WHEELBASE,TRACKWIDTH));
        System.out.println(Arrays.toString(states));
       // assert Arrays.toString(states).equals("[{0.75; 0.25}, {0.75; -0.25}, {1.25; -0.25}, {1.25; 0.25}]");

    }
}
