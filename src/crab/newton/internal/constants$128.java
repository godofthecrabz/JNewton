// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$128 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$128() {}
    public static final FunctionDescriptor NewtonMeshGetEdgeIndices$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetEdgeIndices$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetEdgeIndices",
        constants$128.NewtonMeshGetEdgeIndices$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetFirstFace$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetFirstFace$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetFirstFace",
        constants$128.NewtonMeshGetFirstFace$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetNextFace$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetNextFace$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetNextFace",
        constants$128.NewtonMeshGetNextFace$FUNC
    );
    public static final FunctionDescriptor NewtonMeshIsFaceOpen$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshIsFaceOpen$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshIsFaceOpen",
        constants$128.NewtonMeshIsFaceOpen$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetFaceMaterial$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetFaceMaterial$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetFaceMaterial",
        constants$128.NewtonMeshGetFaceMaterial$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetFaceIndexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetFaceIndexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetFaceIndexCount",
        constants$128.NewtonMeshGetFaceIndexCount$FUNC
    );
}


