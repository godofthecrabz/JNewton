// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$116 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$116() {}
    public static final FunctionDescriptor NewtonMeshFixTJoints$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshFixTJoints$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshFixTJoints",
        constants$116.NewtonMeshFixTJoints$FUNC
    );
    public static final FunctionDescriptor NewtonMeshPolygonize$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshPolygonize$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshPolygonize",
        constants$116.NewtonMeshPolygonize$FUNC
    );
    public static final FunctionDescriptor NewtonMeshTriangulate$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshTriangulate$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshTriangulate",
        constants$116.NewtonMeshTriangulate$FUNC
    );
    public static final FunctionDescriptor NewtonMeshUnion$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshUnion$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshUnion",
        constants$116.NewtonMeshUnion$FUNC
    );
    public static final FunctionDescriptor NewtonMeshDifference$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshDifference$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshDifference",
        constants$116.NewtonMeshDifference$FUNC
    );
    public static final FunctionDescriptor NewtonMeshIntersection$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshIntersection$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshIntersection",
        constants$116.NewtonMeshIntersection$FUNC
    );
}

