// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$101 {

    static final FunctionDescriptor NewtonMeshGetFirstPoint$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetFirstPoint$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetFirstPoint",
        constants$101.NewtonMeshGetFirstPoint$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetNextPoint$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetNextPoint$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetNextPoint",
        constants$101.NewtonMeshGetNextPoint$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetPointIndex$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetPointIndex$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetPointIndex",
        constants$101.NewtonMeshGetPointIndex$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetVertexIndexFromPoint$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetVertexIndexFromPoint$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexIndexFromPoint",
        constants$101.NewtonMeshGetVertexIndexFromPoint$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetFirstEdge$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetFirstEdge$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetFirstEdge",
        constants$101.NewtonMeshGetFirstEdge$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshGetNextEdge$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshGetNextEdge$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetNextEdge",
        constants$101.NewtonMeshGetNextEdge$FUNC, false
    );
}


