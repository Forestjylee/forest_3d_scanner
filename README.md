# forest scanner📸

------

[TOC]

## 0.Abstract

This project is about 3D reconstruction, which contains two main parts. One is [online version](#2Online version), the other is [offline version](#3Offline version). 

Online version has a strong connection with **Intel RealSense D415 depth camera**, which can process RGB images and depth images while collecting them, cost nearly no time to get a raw 3D mesh. If you want to improve the performance of the raw mesh, run `online_optimize_color_map.py` which will costs a few minutes.

Offline version is designed for other cameras or dataset, which is flexible and light. User input merely RGB images, depth images and camera intrinsic matrix, after few minutes, can get a optimized 3D mesh directly.





## 1.Preparation

After you clone the source code from github website of whatever, you have to spend few minutes to check out if you have installed Python(3.6.x), RealSense SDK and  python package dependencies.



### 1.1.Install Python

Download **Python version 3.6.x** from its [official website](https://www.python.org/) and setup it following the instructions.

Or install [anaconda](https://www.anaconda.com/)(Python 3.6.x version).

After installing python, you should add python root folder and $PYTHON_ROOT/scripts to you system environment variables, if you use Mac OS or Linux, you don't need to do that :).

Launch you terminal, input "python" -> "exit()" -> "pip -V":

```powershell
>>> python
Python 3.6.4 |Anaconda custom (64-bit)| (default, Jan 16 2018, 10:22:32) [MSC v.1900 64 bit (AMD64)] on win32
Type "help", "copyright", "credits" or "license" for more information.
>>> exit()
>>> pip -V
pip 20.0.2 from d:\anaconda3\lib\site-packages\pip (python 3.6)
>>>
```

If you get similar output as shown above, you set up environment variable correctly.

> **Note: Please ensure the python interpreter in you computer is version 3.6.x, or it may cause unpredictable bugs.**



### 1.2.Install RealSense SDK

**(If you only use the offline version, you can skip this step.)**

- **Firstly**, download the appropriate exe in official [github repository](https://github.com/IntelRealSense/librealsense/releases). For example, my operating system is Windows 10, so I should download **Intel.RealSense.SDK-WIN10-x.xx.x.xxxx.exe**.
- **Secondly**, execute the .exe file you downloaded, finish the installation following instructions. Don't worry, it's extremely easy.
- **Thirdly**, you have to install the python wrapper for RealSense SDK.

Launch you terminal

```powershell
>>> pip install pyrealsense2
[......]
```

**If it failed, you can download .whl file from [here](https://pypi.org/project/pyrealsense2/#files) to install it.**

I am not sure if RealSense team will simplify the installation process of pyrealsense2, but to the present day (02/18/2020), user need to do extra steps.

Go the folder you install RealSense SDK, copy these files: Intel.Realsense.dll，pyrealsense2.cp36-win_amd64.pyd，realsense2.dll to $PYTHON_ROOT/site-packages/pyrealsense2.

For reference, my target folder is D:\anaconda\Lib\site-packages\pyrealsense2.

Or you can read [this blog](https://blog.csdn.net/pursuit_zhangyu/article/details/84374737), which can be useful.



### 1.3.Install  Python  package dependencies

This project depends on few packages which are easy to installed and don't need to compile.

Launch you terminal

```powershell
>>> cd forest_scanner
>>> pip install -r requirements.txt
[......]
```

If you encounter any problems during installation, try to install that package merely or search solutions on the Internet.

> **Note: If your speed of downloading packages is slow, you can change your pip to download  from mirror source.**

- **Firstly**, installing pqi.
- **Secondly**, use "`pqi use`" to change to a faster mirror source.

```powershell
>>> pip install pqi
[......]
>>> pqi use douban
[......]
```



### 1.4.Others

By the time you get here, the preparation part is almost finishing. If you are a Windows user, maybe you need to install one more stuff, or your camera cannot work.

Go to [this website](https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads) and download the (Visual Studio 2015, 2017 and 2019    vc_redist_x64.exe). If you are 32 bit, download x86 version.

After installing all of these successfully, reboot your computer and the preparation is done.





## 2.Online version



### 2.1.Config parameters

In the online.py, you can see a class named `RealsenseRecorder`. Now, I am gonna explain some important parameter when you initialize it.

```python
def __init__(self, end, output_folder=None, voxel_size=0.0025,
             max_depth_in_meters=1.0, icp_type='point_to_plane', only_body=False):
    """
    @param end: amount of image collection. If you want to scan body, 300 will be fine.
    @param output_folder: the folder that save rgb images and depth images collect by camera. If you don't set its value, images will be saved in $EXECUTE_PATH/dataset.
    @param voxel_size: accuracy of model. According to my test, 0.0025 is a good value to get a good balance of speed and accuracy.
    @param max_depth_in_meters: used for filter depth data. It will only process the pixels in images which are smaller than values set by user.
    @param icp_type: point cloud register methods. "point_to_plane" or "color". The "color" algorithm is under improving, I recommend "point_to_plane".
    @param only_body: only process images with body.
    """
    pass
```



### 2.2.Run

In the main function in `online.py`, you can see:

```python
if __name__ == "__main__":
    recorder = RealsenseRecorder(end=30, icp_type='point_to_plane',
                                 max_depth_in_meters=1.0, voxel_size=0.0025)
    recorder.run()
```

It means it will collect 30 images for reconstruction, you can have a try.

Launch terminal and input

```powershell
>>> python online.py
```

You will see debug information:

```powershell
2020-02-18 11:47:11.376 | DEBUG    | __main__:_init_camera:146 - Init realsense stream pipeline
2020-02-18 11:47:11.826 | DEBUG    | __main__:_init_camera:149 - Get pipeline's config
2020-02-18 11:47:11.828 | DEBUG    | __main__:_init_camera:152 - Start streaming
2020-02-18 11:47:12.514 | DEBUG    | __main__:_get_depth_scale:168 - Init depth sensor
2020-02-18 11:47:12.517 | DEBUG    | __main__:_get_depth_scale:171 - Config depth sensor
2020-02-18 11:47:13.986 | DEBUG    | __main__:run:250 - Clipping distance is 1.0 meter
2020-02-18 11:47:13.988 | DEBUG    | __main__:run:253 - Make clean folder to store color and depth images
2020-02-18 11:48:54.982 | DEBUG    | __main__:_skip_auto_exposure_time:180 - Skip 5 first frames to give the Auto-Exposure time to adjust
2020-02-18 11:48:54.996 | DEBUG    | __main__:_skip_auto_exposure_time:183 - Skip 1 frame
2020-02-18 11:48:54.999 | DEBUG    | __main__:_skip_auto_exposure_time:183 - Skip 2 frame
2020-02-18 11:48:55.031 | DEBUG    | __main__:_skip_auto_exposure_time:183 - Skip 3 frame
2020-02-18 11:48:55.065 | DEBUG    | __main__:_skip_auto_exposure_time:183 - Skip 4 frame
2020-02-18 11:48:55.099 | DEBUG    | __main__:_skip_auto_exposure_time:183 - Skip 5 frame
2020-02-18 11:48:55.171 | DEBUG    | __main__:_init_camera_intrinsic:156 - Saving camera intrinsic info
2020-02-18 11:48:55.179 | DEBUG    | __main__:_init_camera_intrinsic:160 - Saved success
2020-02-18 11:48:55.184 | DEBUG    | __main__:run:283 - Using point to plane ICP registration
2020-02-18 11:48:55.189 | DEBUG    | __main__:run:286 - ----------Start streaming----------
Please press any key to start.
```

If you see such output, it means everything is ready, just press **any key** to continue.

If you use it for the first time, hold you camera and don't move it, observe the speed of processing and you can estimate values you need to set when initializing `class RealsenseRecorder`. After collecting preset amount of images, it will stop automatically. 

> Note: When initialize `class RealsenseRecorder`, set `only_body=True` will cause it only process body images and skip other images.

> Note: During the processing, a window will pump up to show filtered color image in order to make user adjust their camera's position.

Output in terminal:

```powershell
2020-02-18 11:53:51.570 | INFO     | __main__:run:336 - Register frame 29 success
2020-02-18 11:53:51.590 | INFO     | __main__:run:350 - Pose graph has been saved in E:\3d_dev\forest_scanner\pose_graph.json
2020-02-18 11:53:52.197 | INFO     | __main__:run:364 - geometry::TriangleMesh with 17779 points and 31898 triangles.
2020-02-18 11:53:52.214 | INFO     | __main__:run:366 - Result has been saved in E:\3d_dev\forest_scanner\online_raw_mesh.ply
2020-02-18 11:53:53.343 | DEBUG    | __main__:run:371 - ----------Tear down----------
```

From the output, we can see  that three files and a new folder have been created:

- `/dataset`: It contains two sub folders—color and depth. rgb images and depth images save in them respectively.
- `camera_intrinsic.json`: Realsense camera's intrinsic information.
- `pose_graph.json`: Transformation between adjacent images.
- `online_raw_mesh.ply`: 3D mesh without optimization.

If you wanna optimize the mesh, please execute:

```powershell
>>> python online_optimize_color_map.py
[......]
```

Wait a few minutes, a new mesh named online_optimized_mesh.ply is created which save optimized mesh.✨





## 3.Offline version

### 3.1.Arguments

Launch terminal, and then input:

```powershell
>>> python offline.py -h
usage: offline.py [-h] [-d DATASET] [-c CAMERA_INTRINSIC]

optional arguments:
  -h, --help            show this help message and exit
  -d DATASET, --dataset DATASET
                        Dataset folder path, which saves color images and
                        depth images
  -c CAMERA_INTRINSIC, --camera-intrinsic CAMERA_INTRINSIC
                        Camera intrinsic matrix
```

By default, dataset folder is `$EXECUTE_PATH/dataset`, camera intrinsic is `$EXECUTE_PATH/camera_intrisic.json`.



### 3.2.Run

```powershell
>>> python offline.py
[......]
```

Two files are created:

- `offline_raw_mesh.ply`: 3D mesh without optimization.
- `offline_oprimized_mesh.ply`: 3D mesh with optimization.





## 4.Visualization

### 4.1.Normal mesh

`mesh_viewer.py` is used for mesh visualization.

```powershell
>>> python mesh_viewer.py -h
usage: mesh_viewer.py [-h] [-display] [--mesh-file MESH_FILE]

optional arguments:
  -h, --help            show this help message and exit
  -display              Show animation visualization.
  --mesh-file MESH_FILE
                        Mesh file path.
```

You can use "-m" argument to show mesh in a new window.

```powershell
>>> python mesh_viewer.py -m offline_optimized_mesh.ply
```

In addition, use "`-display`" argument to show animation visualization.

```powershell
>> python mesh_viewer.py -display -m offline_optimized_mesh.ply
```



### 4.2.Mesh with bounding box

`bounding_box.py` is used for draw mesh's bounding box.

```powershell
>>> python bounding_box.py -m offline_optimized_mesh.ply
```

What's more, you can use "`-volume`" to calculate bounding box's volume.

```powershell
>>> python bounding_box.py -volume -m offline_optimized_mesh.ply
```

Output:

```powershell
Bounding box's volume:  ???
Length of the bounding box in x, y, and z dimension:  x=???, y=???, z=???
```



Just have a try.🚀





## 5.Others

With a 3D model, you can use it for volume calculation, 3D print and so on. This project mainly use open3d to process point cloud, register them and optimize them. By the way, open3d is such a wonderful package for 3D data processing. If you find any problems or bugs, please email me at 📧forestjylee@qq.com

Enjoy it!😉