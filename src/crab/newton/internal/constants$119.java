// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$119 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$119() {}
    public static final FunctionDescriptor NewtonMeshAddLayer$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMeshAddLayer$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddLayer",
        constants$119.NewtonMeshAddLayer$FUNC
    );
    public static final FunctionDescriptor NewtonMeshAddMaterial$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonMeshAddMaterial$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddMaterial",
        constants$119.NewtonMeshAddMaterial$FUNC
    );
    public static final FunctionDescriptor NewtonMeshAddNormal$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMeshAddNormal$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddNormal",
        constants$119.NewtonMeshAddNormal$FUNC
    );
    public static final FunctionDescriptor NewtonMeshAddBinormal$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMeshAddBinormal$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddBinormal",
        constants$119.NewtonMeshAddBinormal$FUNC
    );
    public static final FunctionDescriptor NewtonMeshAddUV0$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMeshAddUV0$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddUV0",
        constants$119.NewtonMeshAddUV0$FUNC
    );
    public static final FunctionDescriptor NewtonMeshAddUV1$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMeshAddUV1$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshAddUV1",
        constants$119.NewtonMeshAddUV1$FUNC
    );
}


