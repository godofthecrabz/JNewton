// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$124 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$124() {}
    public static final FunctionDescriptor NewtonMeshMaterialGetIndexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetIndexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetIndexCount",
        constants$124.NewtonMeshMaterialGetIndexCount$FUNC
    );
    public static final FunctionDescriptor NewtonMeshMaterialGetIndexStream$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetIndexStream$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetIndexStream",
        constants$124.NewtonMeshMaterialGetIndexStream$FUNC
    );
    public static final FunctionDescriptor NewtonMeshMaterialGetIndexStreamShort$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetIndexStreamShort$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetIndexStreamShort",
        constants$124.NewtonMeshMaterialGetIndexStreamShort$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateFirstSingleSegment$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateFirstSingleSegment$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateFirstSingleSegment",
        constants$124.NewtonMeshCreateFirstSingleSegment$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateNextSingleSegment$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateNextSingleSegment$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateNextSingleSegment",
        constants$124.NewtonMeshCreateNextSingleSegment$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateFirstLayer$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateFirstLayer$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateFirstLayer",
        constants$124.NewtonMeshCreateFirstLayer$FUNC
    );
}

