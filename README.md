# forest scanner📸

------

[TOC]

## 0.Abstract

This project is about 3D reconstruction, which contains two versions. One is online version, the other is offline version. 

Online version only available for **Intel RealSense D-series depth camera**, which can use RGB images and depth images to construct 3D mesh while collecting them, cost nearly no time. If you want to improve the performance of the raw mesh, run `online_optimize_color_map.py` which will costs a few minutes.

Offline version is designed for other cameras or datasets, which is flexible and light. What you need to do is just put RGB images, depth images and camera intrinsic in one folder, run the program and wait for few minutes, then you can get a 3D mesh.





## 1.Preparation

After you clone the source code from [Github website](https://github.com/Forest75/forest_scanner), you have to spend few minutes to check out if you have installed Python(3.6.x), RealSense SDK and  python package dependencies.

Repository's file tree as follow, please check it out.

```shell
forest_scanner
      ├── LICENSE
      ├── README.md
      ├── bounding_box.py
      ├── caffe_object_detection.py
      ├── config.info
      ├── config.ini
      ├── global_config.py
      ├── gui_scanner.py
      ├── mesh_viewer.py
      ├── offline.py
      ├── online.py
      ├── optimize_color_map.py
      ├── preprocess_dataset.py
      ├── requirements.txt
      ├── static
      │   └── models
      │       └── MobileNetSSD
      │           ├── MobileNetSSD_deploy.caffemodel
      │           └── MobileNetSSD_deploy.prototxt
      ├── test_realsense.py
      └── utils.py
```



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
>
> - **Firstly**, installing pqi.
> - **Secondly**, use "`pqi use`" to change to a faster mirror source.
>
> ```powershell
> >>> pip install pqi
> [......]
> >>> pqi use douban
> [......]
> ```
>



### 1.4.Others

By the time you get here, the preparation part is almost finishing. If you are a Windows user, maybe you need to install one more stuff, or your camera cannot work.

Go to [this website](https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads) and download the (Visual Studio 2015, 2017 and 2019    vc_redist_x64.exe). If you are 32 bit, download x86 version.

After installing all of these successfully, reboot your computer and the preparation is done.





## 2.Online version

### 2.0.Test camera

**Firstly**, check whether the connection between camera and computer.(USB 3.0)

**Secondly**, run `test_realsense.py`.

```powershell
>>> python test_realsense.py
```

If everything is installed successfully, there will be a window pump up, which visualizes real-time point cloud.

**Thirdly**, press `Ctrl+c` to terminate it.



### 2.1.Config

In `config.ini` , you can see these parameters

```ini
[online]
debug = no                                                # turn on debug mode,associate with log level
only_person = yes                                         # if only capture images that has person in it
images_count = 50                                         # the amount of images that camera will capture 
voxel_size = 0.0025                                       # the size of one piece of point cloud(you can try 0.002、0.0025、0.003、...)
max_depth_in_meters = 1.0                                 # max validate depth of images
max_correspondence_distance_fine_coeficient = 1.5         # coefficient of ICP algorithm(fine/accuracy alignment)
max_correspondence_distance_coarse_coefficient = 15       # coefficient of ICP algorithm(coarse alignment)
icp_type = point_to_plane                                 # point_to_plane or color
is_output_point_cloud = no                                # if output point cloud file of model
is_optimize_mesh = no                                     # if optimize raw mesh of model(takes few time)
optimization_iteration_count = 200                        # coefficient of optimization algorithm.You can get better result if set it bigger but it will cost more time.
# ouput files name
output_folder = dataset                                   # folder that stores all output files(Please use absolute file path, or it will store in $EXECUTE_PATH/{output_folder})
pose_graph = pose_graph.json                              # pose graph's file name
camera_intrinsic = camera_intrinsic.json                  # camera intrinsic's file name
raw_mesh_filename = online_raw_mesh.ply                   # raw mesh's file name
point_cloud_filename = online_point_cloud.ply             # point cloud's file name
optimized_mesh_filename = online_optimized_mesh.ply       # optimized mesh's file name
```

Feel free to change them~



### 2.2.Run

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

If you see such output, it means everything is ok, just press **any key** to continue.

If you use it for the first time, hold you camera and don't move it, observe the speed of processing and you can estimate values you need to set in `config.ini`. After collecting preset amount of images, it will stop automatically. 

> Note: `only_body=True` will only process human body images and skip other images.

> Note: During the processing, a window will pump up to show filtered color image in order to make user adjust their camera's position.

Output in terminal:

```powershell
2020-02-18 11:53:51.570 | INFO     | __main__:run:336 - Register frame 29 success
2020-02-18 11:53:51.590 | INFO     | __main__:run:350 - Pose graph has been saved in E:\3d_dev\forest_scanner\pose_graph.json
2020-02-18 11:53:52.197 | INFO     | __main__:run:364 - geometry::TriangleMesh with 17779 points and 31898 triangles.
2020-02-18 11:53:52.214 | INFO     | __main__:run:366 - Result has been saved in E:\3d_dev\forest_scanner\online_raw_mesh.ply
2020-02-18 11:53:53.343 | DEBUG    | __main__:run:371 - ----------Tear down----------
```

From the output, we can see  that a new folder have been created:

- `/dataset`: It contains two sub folders—color and depth. RGB images and depth images save in them respectively.
- `/dataset/camera_intrinsic.json`: RealSense camera's intrinsic information.
- `/dataset/pose_graph.json`: Transformation between adjacent images.
- `/dataset/online_raw_mesh.ply`: 3D mesh without optimization.

If you wanna optimize the mesh, please execute:

```powershell
>>> python online_optimize_color_map.py
[......]
```

Wait a few minutes, a new mesh named `online_optimized_mesh.ply` is created which save optimized mesh.✨





## 3.Offline version

Offline version is supposed to input dataset which contains color images and depth images.

Dataset's structure:

```powershell
dataset
│
├─color
│      000000.jpg
│      000001.jpg
│      000002.jpg
│
└─depth
        000000.png
        000001.png
        000002.png
```

> Note: You can rename **dataset folder**'s name unrestrictedly, but **color folder** and **depth folder** cannot be renamed.

### 3.0.Preprocess dataset(Optional)

`preprocess_dataset.py` is used for detecting human body and extract it from input images.

(Actually it support sort of objects, I will offer an interface in the future.)

Usage:

```powershell
>>> python preprocess_dataset.py -h
usage: preprocess_dataset.py [-h] [-o OLD_DATASET] [-n NEW_DATASET]

optional arguments:
  -h, --help            show this help message and exit
  -o OLD_DATASET, --old-dataset OLD_DATASET
                        Dataset needed to be processed.
  -n NEW_DATASET, --new-dataset NEW_DATASET
                        Processed dataset's folder path.
```

Preprocess dataset:

```powershell
>>> python preprocess_dataset.py -o OLD_DATASET_FOLDER_PATH -n NEW_DATASET_FOLDER_PATH
Detect body: E:\3d_dev\my_scanner\forest_scanner\dataset\color\000120.jpg
Detect body: E:\3d_dev\my_scanner\forest_scanner\dataset\color\000121.jpg
Detect body: E:\3d_dev\my_scanner\forest_scanner\dataset\color\000122.jpg
Execute success: E:\3d_dev\my_scanner\forest_scanner\new_dataset\color\000121.jpg
Detect body: E:\3d_dev\my_scanner\forest_scanner\dataset\color\000126.jpg
Execute success: E:\3d_dev\my_scanner\forest_scanner\new_dataset\color\000120.jpg
[......]
```

Body detection and image procession will execute concurrently. If you want to use preprocessed dataset to reconstruct, do not forget to copy `camera_intrinsic.json` to new dataset folder.



### 3.1.Config

As same as online version, you can see some parameters in `config.ini`

```ini
[offline]
debug = no                                                # turn on debug mode,associate with log level
voxel_size = 0.0025                                       # the size of one piece of point cloud(you can try 0.002、0.0025、0.003、...)
max_correspondence_distance_fine_coeficient = 1.5         # coefficient of ICP algorithm(fine/accuracy alignment)
max_correspondence_distance_coarse_coefficient = 15       # coefficient of ICP algorithm(coarse alignment)
is_output_point_cloud = no                                # if output point cloud file of model
is_optimize_mesh = yes                                    # if optimize raw mesh of model(takes few time)
optimization_iteration_count = 200                        # coefficient of optimization algorithm.You can get better result if set it bigger but it will cost more time.
# files name
dataset = dataset                                         # folder that stores all output files(Please use absolute file path, or it will store in $EXECUTE_PATH/{output_folder})
camera_intrinsic = camera_intrinsic.json                  # camera intrinsic's file name needed to be input
pose_graph = pose_graph.json                              # pose graph's file name needed to be input
raw_mesh_filename = offline_raw_mesh.ply                  #  raw mesh's file name
point_cloud_filename = offline_point_cloud.ply            # point cloud's file name
optimized_mesh_filename = offline_optimized_mesh.ply      # optimized mesh's file name
```

Just change them whatever you want.👨‍💻



### 3.2.Run

```powershell
>>> python offline.py
[......]
```

Two files are created in **dataset folder**:

- `offline_raw_mesh.ply`: 3D mesh without optimization.
- `offline_oprimized_mesh.ply`: 3D mesh with optimization.





## 4.Simple GUI

`gui_scanner.py` is a really simple gui for forest_scanner(both online and offline version), based on PySimpleGUI.

> WARNING:  It is under developing and executing only according to `config.ini`, which means input is not used. 

If you are interested in this module, feel free to contact me.





## 5.Visualization

### 5.1.Normal mesh

> Open3D now provides 3D viewer intuitively, you can use it directly.

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



### 5.2.Mesh with bounding box

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

> Note: [Meshlab](http://www.meshlab.net/#download) is also a nice choice for visualizing mesh and point cloud.





## 5.Others

With a 3D model, you can use it for volume calculation, 3D print and so on. This project mainly use open3d to process point cloud, register them and optimize them. If you find any problems or bugs, please email me at 📧forestjylee@qq.com

Enjoy it!😉

