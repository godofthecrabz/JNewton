// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$98 {

    public static final FunctionDescriptor NewtonMeshMaterialGetIndexCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetIndexCount$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetIndexCount",
        constants$98.NewtonMeshMaterialGetIndexCount$FUNC
    );
    public static final FunctionDescriptor NewtonMeshMaterialGetIndexStream$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetIndexStream$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetIndexStream",
        constants$98.NewtonMeshMaterialGetIndexStream$FUNC
    );
    public static final FunctionDescriptor NewtonMeshMaterialGetIndexStreamShort$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetIndexStreamShort$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetIndexStreamShort",
        constants$98.NewtonMeshMaterialGetIndexStreamShort$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateFirstSingleSegment$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateFirstSingleSegment$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateFirstSingleSegment",
        constants$98.NewtonMeshCreateFirstSingleSegment$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateNextSingleSegment$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateNextSingleSegment$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateNextSingleSegment",
        constants$98.NewtonMeshCreateNextSingleSegment$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateFirstLayer$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateFirstLayer$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateFirstLayer",
        constants$98.NewtonMeshCreateFirstLayer$FUNC
    );
}


