# VESC Express Native Library Interface Reference Manual

The VESC Express native library interface allows compiled C libraries shipped
in VESC packages to use selected firmware services. The interface is defined in
`vesc_c_if.h` and is exposed to a native library through `VESC_IF`.

`vesc_c_if` is a stable binary interface. New function pointers are appended to
the end of the table. Existing fields must not be reordered, removed or changed.
An optional function pointer must be checked before it is used. Helpers provided
by `vesc_c_if.h` perform this check where possible.

## LispBM Evaluation


### vesc_lbm_eval

`vesc_lbm_eval` evaluates one LispBM expression and copies the result out of the
LispBM heap. The form is
`vesc_lbm_eval(expression, format, result, result_capacity, result_size, timeout_ms)`.
It is intended for configuration and control operations where evaluator overhead
is acceptable.

The function must be called from a native worker thread created with
`VESC_IF->spawn`. It must not be called directly from a native LispBM extension,
as extensions execute on the evaluator task and waiting there would deadlock the
evaluator.

The arguments are:

   - `expression` is a NUL-terminated string containing one LispBM expression.
   - `format` selects `LBM_EVAL_RESULT_TEXT` or `LBM_EVAL_RESULT_FLAT`.
   - `result` points to caller-owned output storage.
   - `result_capacity` is the size of `result` in bytes.
   - `result_size` optionally receives the number of bytes written. For FLAT
     output it receives the required size if the output buffer is too small.
   - `timeout_ms` is the maximum time the native thread waits for the result.

`LBM_EVAL_RESULT_TEXT` returns the printed LispBM representation. When
`result_capacity` is non-zero the output is always NUL-terminated. This format is
convenient for scalar values and diagnostics.

`LBM_EVAL_RESULT_FLAT` returns the lossless LispBM flat-value representation.
This format should be used for lists, arrays and values that will be passed back
to LispBM.

<table>
<tr>
<td> Example </td> <td> Result </td>
</tr>
<tr>
<td>

```c
char result[16];
size_t result_size = 0;

int status = vesc_lbm_eval(
    "(+ 40 2)",
    LBM_EVAL_RESULT_TEXT,
    result,
    sizeof(result),
    &result_size,
    1000
);
```

</td>
<td>

```c
status == LBM_EVAL_OK
result_size == 2
strcmp(result, "42") == 0
```

</td>
</tr>
</table>




---


### Return Values

`vesc_lbm_eval` returns one of the following status values:

   - `LBM_EVAL_OK` - evaluation completed and the result was copied.
   - `LBM_EVAL_ERROR` - LispBM completed with an evaluation error.
   - `LBM_EVAL_INVALID_ARGUMENT` - an argument or library address is invalid.
   - `LBM_EVAL_NOT_SUPPORTED` - the firmware does not provide the evaluator bridge.
   - `LBM_EVAL_WRONG_CONTEXT` - the function was called on the evaluator task.
   - `LBM_EVAL_BUSY` - all evaluator bridge request slots are in use.
   - `LBM_EVAL_NOT_RUNNING` - the LispBM evaluator is not running.
   - `LBM_EVAL_PAUSE_TIMEOUT` - the evaluator could not be paused in time.
   - `LBM_EVAL_START_FAILED` - LispBM could not create the evaluation context.
   - `LBM_EVAL_TIMEOUT` - the caller stopped waiting before evaluation finished.
   - `LBM_EVAL_RESULT_TOO_SMALL` - the output buffer is missing or too small.
   - `LBM_EVAL_RESULT_UNFLATTENABLE` - the value cannot be serialized as FLAT.
   - `LBM_EVAL_OUT_OF_MEMORY` - firmware memory allocation failed.
   - `LBM_EVAL_ABORTED` - LispBM shutdown interrupted the request.

A timeout only stops the native thread from waiting. It does not terminate the
LispBM context. Firmware-owned request memory remains valid and is released when
the context finishes.

Up to four calls can be in flight. Manual unload is rejected while the library
owns an active call. A LispBM shutdown wakes waiting workers before their native
libraries are stopped.




---


## ABI Design Rules

Native interface additions should follow these rules:

   - Append new function pointers to `vesc_c_if`; never reorder existing slots.
   - Keep ESP-IDF and private firmware structures out of the public header.
   - Copy data into caller-owned storage instead of returning pointers to firmware-owned state.
   - Use versioned snapshots for structures that may grow in future firmware.
   - Use opaque handles with explicit destroy or unsubscribe operations for stateful services.
   - Document callback context, pointer lifetime and ownership.
   - Make unload safe. A library must stop its threads and unregister callbacks
     before its stop function returns.
