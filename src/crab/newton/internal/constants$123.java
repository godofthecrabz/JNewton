// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$123 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$123() {}
    public static final FunctionDescriptor NewtonMeshHasVertexColorChannel$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshHasVertexColorChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshHasVertexColorChannel",
        constants$123.NewtonMeshHasVertexColorChannel$FUNC
    );
    public static final FunctionDescriptor NewtonMeshBeginHandle$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshBeginHandle$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshBeginHandle",
        constants$123.NewtonMeshBeginHandle$FUNC
    );
    public static final FunctionDescriptor NewtonMeshEndHandle$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshEndHandle$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshEndHandle",
        constants$123.NewtonMeshEndHandle$FUNC
    );
    public static final FunctionDescriptor NewtonMeshFirstMaterial$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshFirstMaterial$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshFirstMaterial",
        constants$123.NewtonMeshFirstMaterial$FUNC
    );
    public static final FunctionDescriptor NewtonMeshNextMaterial$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMeshNextMaterial$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshNextMaterial",
        constants$123.NewtonMeshNextMaterial$FUNC
    );
    public static final FunctionDescriptor NewtonMeshMaterialGetMaterial$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMeshMaterialGetMaterial$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshMaterialGetMaterial",
        constants$123.NewtonMeshMaterialGetMaterial$FUNC
    );
}


