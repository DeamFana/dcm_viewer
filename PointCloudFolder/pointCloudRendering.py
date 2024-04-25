import numpy as np
import open3d as o3d
import queue
import threading
from os import path

THRESHOLD_MIN = 60
THRESHOLD_MAX = 61

def changeTHRESHOLD(delta):
    global THRESHOLD_MIN, THRESHOLD_MAX
    THRESHOLD_MIN = THRESHOLD_MIN + delta
    THRESHOLD_MAX = THRESHOLD_MAX + delta
    print("THRESHOLD_MIN: ", THRESHOLD_MIN)
    print("THRESHOLD_MAX: ", THRESHOLD_MAX)

def transform(ndarray, n):

    with open(f"test{n}.pcd", "w") as f:
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")

        if len(ndarray.shape) < 3:
            raise ValueError("Input array must be 3-Dimension")
        else:
            result = []
            
            height = ndarray.shape[0]
            all_width = ndarray.shape[1]*ndarray.shape[2]
            all_points = height*all_width
            points = 0

            f.write("WIDTH {}\n".format(all_width))
            f.write("HEIGHT {}\n".format(height))
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")


            for z in range(ndarray.shape[0]):
                # print("{:.1f}%".format(z*100/ndarray.shape[0]))
                for y in range(ndarray.shape[1]):
                    for x in range(ndarray.shape[2]):
                        if ndarray[z, y, x] >= THRESHOLD_MIN and ndarray[z, y, x] <= THRESHOLD_MAX:
                            result.append((float((x-ndarray.shape[2])/100),float((y-ndarray.shape[1])/100),float((z-ndarray.shape[0])/100), ndarray[z,y,x]))
                            points = points + 1
            
            f.write("POINTS {}\n".format(points))
            f.write("DATA ascii\n")
            for i in result:
                f.write("{} {} {} {}\n".format(i[0], i[1], i[2], i[3]))

def npy2pcd():
    volume_ndarray = np.load('./3.npy', allow_pickle=False)
    for i in range(50):
        changeTHRESHOLD(1)
        transform(volume_ndarray, i)
    # rendering()
    print("Done")


def rendering():
    PATH = "./test19.pcd"

    # --------------------------- 加载点云 ---------------------------
    print("->正在加载点云... ")
    pcd = o3d.io.read_point_cloud(PATH)
    print("原始点云：", pcd)

    print("->正在体素下采样...")
    voxel_size = 0.01
    downpcd = pcd.voxel_down_sample(voxel_size)
    print(downpcd)

    alpha = 0.008
    print(f"alpha={alpha:.3f}")
    radius = 0.3      # 搜索半径
    max_nn = 100         # 邻域内用于估算法线的最大点数
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))     # 执行法线估计

    # o3d.visualization.draw_geometries([downpcd], mesh_show_back_face=True)

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(downpcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


def visualize_pointscloud(show_q):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=800, height=600)
    pointcloud = o3d.geometry.PointCloud()
    to_reset = True
    vis.add_geometry(pointcloud)

    while True:
        try:
            pcd = show_q.get()

            pcd = np.asarray(pcd.points).reshape((-1, 3))
            pointcloud.points = o3d.utility.Vector3dVector(pcd)
            # vis.update_geometry()
            # 注意，如果使用的是open3d 0.8.0以后的版本，这句话应该改为下面格式
            vis.update_geometry(pointcloud)
            if to_reset:
                vis.reset_view_point(True)
                to_reset = False
            vis.poll_events()
            vis.update_renderer()
        except:
            print("error")
            continue

def continuousRendering():

    show_q = queue.Queue(1)
    visual = threading.Thread(target=visualize_pointscloud, args=(show_q,))
    visual.start()

    input_dir = r"./"
    frame = 0
    while True:
        input_name = path.join(input_dir, "test" + str(frame)+".pcd")
        print(input_name)
        # 获取雷达数据
        pcd = o3d.io.read_point_cloud(input_name)
        print("原始点云：", pcd)

        if show_q.full():
            print("原始点云：", show_q.get())
            
        show_q.put(pcd)
        frame += 1  # 迭代读取下一张图片
        frame %= 49  # 由于文件夹中最多只有49图片，超出了，又会回到0,循环


rendering()