// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$25 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$25() {}
    public static final FunctionDescriptor NewtonUpdateAsync$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonUpdateAsync$MH = RuntimeHelper.downcallHandle(
        "NewtonUpdateAsync",
        constants$25.NewtonUpdateAsync$FUNC
    );
    public static final FunctionDescriptor NewtonWaitForUpdateToFinish$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWaitForUpdateToFinish$MH = RuntimeHelper.downcallHandle(
        "NewtonWaitForUpdateToFinish",
        constants$25.NewtonWaitForUpdateToFinish$FUNC
    );
    public static final FunctionDescriptor NewtonGetNumberOfSubsteps$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonGetNumberOfSubsteps$MH = RuntimeHelper.downcallHandle(
        "NewtonGetNumberOfSubsteps",
        constants$25.NewtonGetNumberOfSubsteps$FUNC
    );
    public static final FunctionDescriptor NewtonSetNumberOfSubsteps$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonSetNumberOfSubsteps$MH = RuntimeHelper.downcallHandle(
        "NewtonSetNumberOfSubsteps",
        constants$25.NewtonSetNumberOfSubsteps$FUNC
    );
    public static final FunctionDescriptor NewtonGetLastUpdateTime$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonGetLastUpdateTime$MH = RuntimeHelper.downcallHandle(
        "NewtonGetLastUpdateTime",
        constants$25.NewtonGetLastUpdateTime$FUNC
    );
    public static final FunctionDescriptor NewtonSerializeToFile$bodyCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonSerializeToFile$bodyCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSerializeToFile$bodyCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonSerializeToFile$bodyCallback.class, "apply", constants$25.NewtonSerializeToFile$bodyCallback_UP$FUNC);
}


