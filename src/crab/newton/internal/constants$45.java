// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$45 {

	public static final FunctionDescriptor NewtonFracturedCompoundNeighborNodeList$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundNeighborNodeList$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundNeighborNodeList",
        constants$45.NewtonFracturedCompoundNeighborNodeList$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundGetMainMesh$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundGetMainMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundGetMainMesh",
        constants$45.NewtonFracturedCompoundGetMainMesh$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundGetFirstSubMesh$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundGetFirstSubMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundGetFirstSubMesh",
        constants$45.NewtonFracturedCompoundGetFirstSubMesh$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundGetNextSubMesh$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundGetNextSubMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundGetNextSubMesh",
        constants$45.NewtonFracturedCompoundGetNextSubMesh$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundCollisionGetVertexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundCollisionGetVertexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundCollisionGetVertexCount",
        constants$45.NewtonFracturedCompoundCollisionGetVertexCount$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundCollisionGetVertexPositions$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundCollisionGetVertexPositions$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundCollisionGetVertexPositions",
        constants$45.NewtonFracturedCompoundCollisionGetVertexPositions$FUNC, false
    );
}

