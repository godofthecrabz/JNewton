// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$22 {

    public static final FunctionDescriptor NewtonSetContactMergeTolerance$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonSetContactMergeTolerance$MH = RuntimeHelper.downcallHandle(
        "NewtonSetContactMergeTolerance",
        constants$22.NewtonSetContactMergeTolerance$FUNC
    );
    public static final FunctionDescriptor NewtonInvalidateCache$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonInvalidateCache$MH = RuntimeHelper.downcallHandle(
        "NewtonInvalidateCache",
        constants$22.NewtonInvalidateCache$FUNC
    );
    public static final FunctionDescriptor NewtonSetSolverIterations$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonSetSolverIterations$MH = RuntimeHelper.downcallHandle(
        "NewtonSetSolverIterations",
        constants$22.NewtonSetSolverIterations$FUNC
    );
    public static final FunctionDescriptor NewtonGetSolverIterations$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonGetSolverIterations$MH = RuntimeHelper.downcallHandle(
        "NewtonGetSolverIterations",
        constants$22.NewtonGetSolverIterations$FUNC
    );
    public static final FunctionDescriptor NewtonSetParallelSolverOnLargeIsland$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonSetParallelSolverOnLargeIsland$MH = RuntimeHelper.downcallHandle(
        "NewtonSetParallelSolverOnLargeIsland",
        constants$22.NewtonSetParallelSolverOnLargeIsland$FUNC
    );
    public static final FunctionDescriptor NewtonGetParallelSolverOnLargeIsland$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonGetParallelSolverOnLargeIsland$MH = RuntimeHelper.downcallHandle(
        "NewtonGetParallelSolverOnLargeIsland",
        constants$22.NewtonGetParallelSolverOnLargeIsland$FUNC
    );
}


