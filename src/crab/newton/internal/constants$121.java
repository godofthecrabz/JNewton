// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$121 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$121() {}
    public static final FunctionDescriptor NewtonMeshGetIndexToVertexMap$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetIndexToVertexMap$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetIndexToVertexMap",
        constants$121.NewtonMeshGetIndexToVertexMap$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetVertexDoubleChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetVertexDoubleChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexDoubleChannel",
        constants$121.NewtonMeshGetVertexDoubleChannel$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetVertexChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetVertexChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexChannel",
        constants$121.NewtonMeshGetVertexChannel$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetNormalChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetNormalChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetNormalChannel",
        constants$121.NewtonMeshGetNormalChannel$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetBinormalChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetBinormalChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetBinormalChannel",
        constants$121.NewtonMeshGetBinormalChannel$FUNC
    );
    public static final FunctionDescriptor NewtonMeshGetUV0Channel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshGetUV0Channel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetUV0Channel",
        constants$121.NewtonMeshGetUV0Channel$FUNC
    );
}


