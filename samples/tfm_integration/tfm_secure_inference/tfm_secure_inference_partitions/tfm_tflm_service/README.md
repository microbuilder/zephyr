# Tensorflow lite-micro as TF-M secure service

TensorFlow Lite for Microcontrollers (TFLM) is designed to run machine learning models on microcontrollers and other devices with only few kilobytes of memory. The core runtime just fits in 16 KB on an Arm Cortex M3 and can run many basic models. [ref](https://www.tensorflow.org/lite/microcontrollers).

Description about the structure of [repository](https://github.com/tensorflow/tflite-micro/tree/main/tensorflow/lite/micro) can be found [here](https://www.tensorflow.org/lite/microcontrollers/library).

This work is still in prototyping stage, therefore the code is residing in my fork of Zephyr and TF-M.

Zephyr fork: https://github.com/urutva/zephyr
branch: [tfm-example-partition-tflm](https://github.com/urutva/zephyr/tree/tfm-example-partition-tflm)

TF-M fork: https://github.com/urutva/zephyr-trusted-firmware-m
branch: [tfm-example-partition-tflm](https://github.com/urutva/zephyr-trusted-firmware-m/tree/tfm-example-partition-tflm)

## Zephyr setup
Setting up environment required to build Zephyr is described [here](https://docs.zephyrproject.org/latest/getting_started/index.html). Use my [fork of Zephyr](https://github.com/urutva/zephyr/tree/tfm-example-partition-tflm) instead of upstream Zephyr.

If you have followed all the instructions correctly, then you should be able to build blinky example using `west build -p auto -b <your-board-name> samples/basic/blinky`.

## Exporting tensorflow lite-micro

Tensorflow lite-micro provides a [python script](https://github.com/tensorflow/tflite-micro/blob/main/tensorflow/lite/micro/tools/project_generation/create_tflm_tree.py) to export sources without build system. However, in order to build TFLM with TF-M, we need to add `CMakeLists.txt`. Currently, this is done manually, but it can be automated using a python script.

```bash
git clone git@github.com:tensorflow/tflite-micro.git
cd tflite-micro

python3 tensorflow/lite/micro/tools/project_generation/create_tflm_tree.py \
        -e hello_world \
        /tmp/tflm-tree
```

After successful execution of the script, TFLM source and [hello_world](https://github.com/tensorflow/tflite-micro/tree/main/tensorflow/lite/micro/examples/hello_world) example can be found in `/tmp/tflm-tree`.

### Copy TFLM runtime to TF-M

```bash
cp /tmp/tflm-tree/tensorflow path/to/zephyrproject/modules/tee/tfm/trusted-firmware-m/secure_fw/partitions/example_partition/tflm/
cp /tmp/tflm-tree/third_party path/to/zephyrproject/modules/tee/tfm/trusted-firmware-m/secure_fw/partitions/example_partition/tflm/
```

Modify `CMakeLists.txt` in `path/to/zephyrproject/modules/tee/tfm/trusted-firmware-m/secure_fw/partitions/example_partition/tflm/` if necessary.

### Copy TFLM example to TF-M

```bash
cp /tmp/tflm-tree/examples/hello_world/* path/to/zephyrproject/modules/tee/tfm/trusted-firmware-m/secure_fw/partitions/example_partition/hello_world
```

Ensure that necessary changes are done so that the example can run as a secure service. Modify `CMakeLists.txt` in `path/to/zephyrproject/modules/tee/tfm/trusted-firmware-m/secure_fw/partitions/example_partition/hello_world` if necessary.

## Zephyr NS sample

Add/update Zephyr NS sample (for example `path/to/zephyrproject/samples/tfm_integration/psa_custom_service`) that can invoke the secure service and handle the output.

## Build and run

Build:
```bash
west build -p -b mps2_an521_ns samples/tfm_integration/psa_custom_service
```

Run:
```bash
qemu-system-arm -M mps2-an521 -device loader,file=./build/tfm_merged.hex -serial stdio \
  -monitor tcp:localhost:4444,server,nowait \
  -device lsm303dlhc_mag,id=lsm303,address=0x1E
```

Expected output:
```bash
...
[00:00:00.046,000] <inf> app: Get sine value using secure inference
[Example partition] Starting secure inferencing...
Model: Sine of 0 deg is: 0.000000	C Mathlib: Sine of 0 deg is: 0.000000	Deviation: 0.000000
[Example partition] Starting secure inferencing...
Model: Sine of 1 deg is: 0.016944	C Mathlib: Sine of 1 deg is: 0.017452	Deviation: 0.000508
[Example partition] Starting secure inferencing...
Model: Sine of 2 deg is: 0.059304	C Mathlib: Sine of 2 deg is: 0.034899	Deviation: 0.024405
[Example partition] Starting secure inferencing...
Model: Sine of 3 deg is: 0.101664	C Mathlib: Sine of 3 deg is: 0.052336	Deviation: 0.049328
[Example partition] Starting secure inferencing...
Model: Sine of 4 deg is: 0.101664	C Mathlib: Sine of 4 deg is: 0.069756	Deviation: 0.031908
[Example partition] Starting secure inferencing...
Model: Sine of 5 deg is: 0.101664	C Mathlib: Sine of 5 deg is: 0.087156	Deviation: 0.014508
[Example partition] Starting secure inferencing...
Model: Sine of 6 deg is: 0.160968	C Mathlib: Sine of 6 deg is: 0.104528	Deviation: 0.056440
[Example partition] Starting secure inferencing...
Model: Sine of 7 deg is: 0.160968	C Mathlib: Sine of 7 deg is: 0.121869	Deviation: 0.039099
[Example partition] Starting secure inferencing...
Model: Sine of 8 deg is: 0.186384	C Mathlib: Sine of 8 deg is: 0.139173	Deviation: 0.047211
[Example partition] Starting secure inferencing...
Model: Sine of 9 deg is: 0.203328	C Mathlib: Sine of 9 deg is: 0.156434	Deviation: 0.046894
[Example partition] Starting secure inferencing...
Model: Sine of 10 deg is: 0.237216	C Mathlib: Sine of 10 deg is: 0.173648	Deviation: 0.063568
...
````


## Debugging
The size of TF-M + TFLM is higher than the memory allocated to TF-M when debugging is enabled. In order to debug both TF-M + TFLM and Zephyr, we need to modify linker scripts to increase the memory allocated to TF-M at the same time reducing the memory allocated to Zephyr.

Zephyr:
```bash
--- a/boards/arm/mps2_an521/mps2_an521_ns.dts
+++ b/boards/arm/mps2_an521/mps2_an521_ns.dts
@@ -105,8 +105,8 @@
         * https://git.trustedfirmware.org/TF-M/trusted-firmware-m.git/tree/platform/ext/target/mps2/an521/partition/flash_layout.h
         */

-       code: memory@100000 {
-           reg = <0x00100000 DT_SIZE_K(512)>;
+       code: memory@140000 {
+           reg = <0x00140000 DT_SIZE_K(256)>;
```

TF-M:
```
--- a/trusted-firmware-m/platform/ext/target/arm/mps2/an521/partition/flash_layout.h
+++ b/trusted-firmware-m/platform/ext/target/arm/mps2/an521/partition/flash_layout.h
@@ -60,8 +60,8 @@
  */

 /* Size of a Secure and of a Non-secure image */
-#define FLASH_S_PARTITION_SIZE          (0x80000) /* S partition: 512 KB */
-#define FLASH_NS_PARTITION_SIZE         (0x80000) /* NS partition: 512 KB */
+#define FLASH_S_PARTITION_SIZE          (0xC0000) /* S partition: 768 KB */
+#define FLASH_NS_PARTITION_SIZE         (0x40000) /* NS partition: 256 KB */
 #define FLASH_MAX_PARTITION_SIZE        ((FLASH_S_PARTITION_SIZE >   \
                                           FLASH_NS_PARTITION_SIZE) ? \
                                          FLASH_S_PARTITION_SIZE :    \
```

## Observations
The linker variable `__exidx_end` is not defined for `TFM_LVL == 1`, however, adding TFLM causes build failure due to missing `__exidx_end`. We need to check this with TF-M.

```bash
--- a/trusted-firmware-m/platform/ext/common/gcc/tfm_common_s.ld
+++ b/trusted-firmware-m/platform/ext/common/gcc/tfm_common_s.ld
@@ -183,7 +183,7 @@ SECTIONS
     Image$$ER_CODE_SRAM$$Limit = ADDR(.ER_CODE_SRAM) + SIZEOF(.ER_CODE_SRAM);
 #endif

-#if TFM_LVL != 1
+/* #if TFM_LVL != 1 */
     .ARM.extab :
     {
         *(.ARM.extab* .gnu.linkonce.armextab.*)
@@ -196,7 +196,7 @@ SECTIONS
     } > FLASH
     __exidx_end = .;

-#endif /* TFM_LVL != 1 */
+/* #endif TFM_LVL != 1 */
```