// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$92 {

    static final FunctionDescriptor NewtonMeshSimplify$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshSimplify$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshSimplify",
        constants$92.NewtonMeshSimplify$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshApproximateConvexDecomposition$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshApproximateConvexDecomposition$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshApproximateConvexDecomposition",
        constants$92.NewtonMeshApproximateConvexDecomposition$FUNC, false
    );
    static final FunctionDescriptor NewtonRemoveUnusedVertices$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonRemoveUnusedVertices$MH = RuntimeHelper.downcallHandle(
        "NewtonRemoveUnusedVertices",
        constants$92.NewtonRemoveUnusedVertices$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshBeginBuild$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshBeginBuild$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshBeginBuild",
        constants$92.NewtonMeshBeginBuild$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshBeginFace$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshBeginFace$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshBeginFace",
        constants$92.NewtonMeshBeginFace$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshAddPoint$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_DOUBLE$LAYOUT,
        Constants$root.C_DOUBLE$LAYOUT,
        Constants$root.C_DOUBLE$LAYOUT
    );
    static final MethodHandle NewtonMeshAddPoint$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddPoint",
        constants$92.NewtonMeshAddPoint$FUNC, false
    );
}


