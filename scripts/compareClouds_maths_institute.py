# examples/Python/Basic/pointcloud.py

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import copy
import pandas as pd
from scipy.spatial.transform import Rotation


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def calculate_chamfer_distance(pc, pc_gt):

    chamfer_distance_threshold = 20.0
    dist_pc1_pc2 = pc_gt.compute_point_cloud_distance(pc)
    # dist_pc1_pc2 is an Open3d object, we need to convert it to a numpy array to 
    dist_pc1_pc2 = np.asarray(dist_pc1_pc2)
    # remove too big distances
    dist_pc1_pc2 = dist_pc1_pc2[(dist_pc1_pc2 < chamfer_distance_threshold)]

    # Calculate distances of pc_gt to pc. 
    dist_pc2_pc1 = pc.compute_point_cloud_distance(pc_gt)
    # dist_2c1_1c2 is an Open3d object, we need to convert it to a numpy array to 
    dist_pc2_pc1 = np.asarray(dist_pc2_pc1)
    # remov2 to1 big distances
    dist_pc2_pc1 = dist_pc2_pc1[(dist_pc2_pc1 < chamfer_distance_threshold)]

    df = pd.DataFrame({"distances": dist_pc1_pc2})
    # Some graphs
    # ax1 = df.boxplot(return_type="axes") # BOXPLOT
    # ax2 = df.plot(kind="hist", alpha=0.5, bins = 1000) # HISTOGRAM
    # ax3 = df.plot(kind="line") # SERIE
    # plt.show()

    chamfer_dst = (sum(dist_pc1_pc2) + sum(dist_pc1_pc2)) / (len(dist_pc1_pc2) + len(dist_pc2_pc1))
    
    return chamfer_dst


def evaluate_pointclouds(pc, pc_gt, pc_name, gt_name):
    source = copy.deepcopy(pc)
    target = copy.deepcopy(pc_gt)
    eval_thresh1 = 0.4
    eval_thresh2 = 9999
    icp_thresh = 200
    trans_init = np.eye(4,4)
    angles = [70,0,0]
    t = np.array([5,0,0])
    r = Rotation.from_euler("zyx",angles,degrees=True)
    trans_init[:3,:3] = r.as_matrix()
    trans_init[:3,3] = t
    trans_init = np.linalg.inv(trans_init)

    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, icp_thresh, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))

    evaluation = o3d.pipelines.registration.evaluate_registration(source, target, eval_thresh1, reg_p2l.transformation)
    print()
    print('\033[93m', 'GT:', gt_name, ' Points:', len(pc_gt.points), ' | User:', pc_name, ' Points:', len(pc.points))
    print('\033[92m', "Source: User | Target: GT | Thresh: " + str(eval_thresh1) + " | Point-to-plane")
    print('\033[0m', str(evaluation).split("Access")[0])

    evaluation = o3d.pipelines.registration.evaluate_registration(source, target, eval_thresh2, reg_p2l.transformation)
    print('\033[92m', "Source: User | Target: GT | Thresh: " + str(eval_thresh2) + " | Point-to-plane")
    print('\033[0m', str(evaluation).split("Access")[0])

    evaluation = o3d.pipelines.registration.evaluate_registration(target,source , eval_thresh1, np.linalg.inv(reg_p2l.transformation))
    print('\033[92m', "Source: GT | Target: User | Thresh: " + str(eval_thresh1) + " | Point-to-plane")
    print('\033[0m', str(evaluation).split("Access")[0])

    evaluation = o3d.pipelines.registration.evaluate_registration(target, source, eval_thresh2, np.linalg.inv(reg_p2l.transformation))
    print('\033[92m', "Source: GT | Target: User | Thresh: " + str(eval_thresh2) + " | Point-to-plane")
    print('\033[0m', str(evaluation).split("Access")[0])

    # source.transform(reg_p2l.transformation)
    # chamfer_dst = calculate_chamfer_distance(pc, pc_gt)
    # print('\033[94m' + " Chamfer distance: " + str(chamfer_dst))
    print('\033[0m')
    

if __name__ == "__main__":
    # Folder paths
    gt_path = '../gt/maps/maths_institute/'
    user_path = '../output/pcd/'
    # Names of GT maps
    gt_maps_names = ['maths-institute.ply']
    # Names of user maps
    user_maps_names = ['surfelCloud_0.pcd', 'surfelCloud_5.pcd']

    for i, gt_map_name in enumerate(gt_maps_names):
        gt_map_pcd = o3d.io.read_point_cloud(gt_path + gt_map_name)
        for j, user_map_name in enumerate(user_maps_names):
            user_map_pcd = o3d.io.read_point_cloud(user_path + user_map_name)
            evaluate_pointclouds(user_map_pcd, gt_map_pcd, user_map_name, gt_map_name)




