# Android 传感器坐标系统
* * *

在 Android 系统里，如果使用传感器，需要注意存在 3 种坐标系：
1. 传感器坐标系 (设备参照系)。
2. 世界参照系 (Android ENU)。
3. CardBoard 坐标系。

# [传感器坐标系](https://developer.android.com/guide/topics/sensors/sensors_overview#sensors-coords)

通常，传感器框架使用标准的 3 轴坐标系来表示数据值。对于大多数传感器，当设备处于默认屏幕方向时，会相对于设备屏幕来定义坐标系（参见图 1）。当设备处于默认屏幕方向时，X 轴为水平向右延伸，Y 轴为垂直向上延伸，Z 轴为垂直于屏幕向外延伸。在此坐标系中，屏幕后面的坐标将具有负 Z 值。以下传感器使用此坐标系：

* [加速度传感器](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-accel)
* [重力传感器](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-grav)
* [陀螺仪](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-gyro)
* [线性加速度传感器](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-linear)
* [地磁场传感器](https://developer.android.com/guide/topics/sensors/sensors_position.html#sensors-pos-mag)

![图 1](https://developer.android.com/images/axis_device.png)

图 1. Sensor API 使用的坐标系（相对于设备）。

关于此坐标系，特别需要注意的一点就是当设备的屏幕方向改变时，坐标轴不会转换，也就是说，传感器的坐标系不会随着设备的移动而改变。此行为与 OpenGL 坐标系的行为是相同的。

还需要注意的一点是，你的应用不能假设设备的自然（默认）屏幕方向是竖屏。许多平板设备的自然屏幕方向为横屏。传感器坐标系始终基于设备的**自然屏幕**方向。

最后，如果你的应用将传感器数据对应到屏幕显示，你需要使用 [getRotation()](https://developer.android.com/reference/android/view/Display.html#getRotation()) 方法确定屏幕的旋转度，然后使用 [remapCoordinateSystem()](https://developer.android.com/reference/android/hardware/SensorManager.html#remapCoordinateSystem(float[],%20int,%20int,%20float[])) 方法将传感器坐标映射到屏幕坐标。即使你的清单指定了仅限竖屏显示，你也需要这样做。

! **注意**：有些传感器和方法使用的坐标系基于世界参照系（而不是设备参照系）。这些传感器和方法返回的数据表示设备相对于地球的运动或位置。如需了解详情，请参阅 [getOrientation()](https://developer.android.com/reference/android/hardware/SensorManager.html#getOrientation(float[],%20float[])) 方法、[getRotationMatrix()](https://developer.android.com/reference/android/hardware/SensorManager.html#getRotationMatrix(float[],%20float[],%20float[],%20float[])) 方法、[屏幕方向传感器](https://developer.android.com/guide/topics/sensors/sensors_position.html#sensors-pos-orient)和[旋转矢量传感器](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-rotate)。

# [旋转矢量坐标系](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-rotate)

旋转矢量将设备的屏幕方向表示为角度和轴的组合，其中设备已围绕轴（x、y 或 z）旋转了 θ 度。

旋转矢量的三个元素表示如下：

![vector of quaternion](https://developer.android.com/images/guide/topics/sensors/rotational_vec.png)

其中旋转矢量的大小等于 sin(θ/2)，并且旋转矢量的方向等于旋转轴的方向。

![图 2](https://developer.android.com/images/axis_globe.png)

图 2. 旋转矢量传感器使用的坐标系。

旋转矢量的三个元素等于单位四元数（cos(θ/2)、x*sin(θ/2)、y*sin(θ/2)、z*sin(θ/2)）的最后三个分量。旋转矢量的元素没有单位。x、y 和 z 轴的定义方法与加速传感器的定义方法相同。参考坐标系被定义为直接正交基（见图 2）。该坐标系具有以下特征：

* X 定义为矢量积 Y x Z。其在设备当前位置与地面相切，并大约指向东。
* Y 在设备当前位置与地面相切，并指向地磁北极。
* Z 指向天空并与地平面垂直。

有关展示如何使用旋转矢量传感器的示例应用，请参阅 [RotationVectorDemo.java](https://android.googlesource.com/platform/development/+/master/samples/ApiDemos/src/com/example/android/apis/os/RotationVectorDemo.java)。 

# CardBoard 坐标系

![图 3](https://github.com/shuyong/Google-CardBoard-Sensor-Fusion/blob/master/doc/cardboard-ypr.jpg)

# 本项目的坐标系

在以上 3 种坐标系中，相互关系是这样的：
* 人在所处的大地水平面上，面朝北方 (N-Y)，右边为东方 (E-X)，头顶向上 (U-Z) 为测量重力矢量。这就是 图 2 所示的 Android ENU 坐标系。
* 人手举竖屏手机，右手按住电源键，该方向为 X 轴。垂直向上为 Y 轴。屏幕朝向人脸为 Z 轴。这就是 图 1 所示的设备坐标系。人脸朝向北方时为 (0,0,0) 初始状态。
* CardBoard 坐标系和 Android 设备坐标系不同的地方在于手机是横屏，电源键向上。

如果要用欧拉角表示设备状态，要注意 Android 设备的坐标轴表示和[一般的资料](https://en.wikipedia.org/wiki/Euler_angles)里的坐标轴表示不一样：
* 在一般资料里，绕 Z 轴旋转的角度为 Yaw，绕 Y 轴旋转的角度为 Pitch，绕 X 轴旋转的角度为 Roll。
* 在 Android 设备坐标系里，绕 Y 轴旋转的角度为 Yaw，绕 X 轴旋转的角度为 Pitch，绕 Z 轴旋转的角度为 Roll。也就是，人举着手机朝着 -Z 轴方向前进，绕 Y 轴旋转寻找方向。
* 在 CardBoard 坐标系里，绕 X 轴旋转的角度为 Yaw，绕 Y 轴旋转的角度为 Pitch，绕 Z 轴旋转的角度为 Roll。因为手机是横屏，电源键( X 轴)向上。也就是，人戴着手机朝着 -Z 轴方向前进，绕 X 轴旋转寻找方向。

因为手举着手机采集数据太累。而且手有抖动，也不精确，不利于采集数据，分析算法。所以在本项目里我修改了坐标系。这是我对 Sensor Fusion 算法唯一修改的地方。

手机平放在水平桌面上：
* 指向右边电源键为 X 轴。绕 X 轴旋转为 Roll。
* 指向机头为 Y 轴。绕 Y 轴旋转的角度为 Pitch。
* 屏幕朝向天空为 Z 轴。绕 Z 轴旋转的角度为 Yaw。

# 参考文献

1. [Sensor Coordinate System](https://developer.android.com/guide/topics/sensors/sensors_overview#sensors-coords)
1. [Use the rotation vector sensor](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-rotate)
1. [Sensors Overview](https://developer.android.com/guide/topics/sensors/sensors_overview)
1. [Motion sensors](https://developer.android.com/guide/topics/sensors/sensors_motion.html)
1. [Position sensors](https://developer.android.com/guide/topics/sensors/sensors_position.html)
1. [Sensor stack](https://source.android.com/devices/sensors/sensor-stack)
1. [Hardware/Sensor](https://developer.android.com/reference/android/hardware/Sensor.html)

