// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$95 {

	public static final FunctionDescriptor NewtonMeshGetIndexToVertexMap$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshGetIndexToVertexMap$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetIndexToVertexMap",
        constants$95.NewtonMeshGetIndexToVertexMap$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshGetVertexDoubleChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshGetVertexDoubleChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexDoubleChannel",
        constants$95.NewtonMeshGetVertexDoubleChannel$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshGetVertexChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshGetVertexChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetVertexChannel",
        constants$95.NewtonMeshGetVertexChannel$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshGetNormalChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshGetNormalChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetNormalChannel",
        constants$95.NewtonMeshGetNormalChannel$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshGetBinormalChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshGetBinormalChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetBinormalChannel",
        constants$95.NewtonMeshGetBinormalChannel$FUNC, false
    );
	public static final FunctionDescriptor NewtonMeshGetUV0Channel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonMeshGetUV0Channel$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshGetUV0Channel",
        constants$95.NewtonMeshGetUV0Channel$FUNC, false
    );
}


