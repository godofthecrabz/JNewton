// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$90 {

	public static final FunctionDescriptor NewtonMeshOptimize$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshOptimize$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshOptimize",
        constants$90.NewtonMeshOptimize$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshOptimizePoints$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshOptimizePoints$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshOptimizePoints",
        constants$90.NewtonMeshOptimizePoints$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshOptimizeVertex$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshOptimizeVertex$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshOptimizeVertex",
        constants$90.NewtonMeshOptimizeVertex$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshIsOpenMesh$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshIsOpenMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshIsOpenMesh",
        constants$90.NewtonMeshIsOpenMesh$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshFixTJoints$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshFixTJoints$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshFixTJoints",
        constants$90.NewtonMeshFixTJoints$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshPolygonize$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshPolygonize$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshPolygonize",
        constants$90.NewtonMeshPolygonize$FUNC, false
    );
}


