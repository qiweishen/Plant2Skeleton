import os
import shutil
from typing import Tuple
from plyfile import PlyData, PlyElement

from evaluation.semantic import semantic_evaluation
from evaluation.instance import instance_evaluation


def get_total_pred_gt(pre_folder_path: str, gt_folder_path: str) -> Tuple[list, list, list, list, list]:
    """
    :param pre_folder_path:
    :param gt_folder_path:
    :return: Merged [pred sematic_label, pred instance_label, gt sematic_label, gt instance_label, file_names]
    """
    merged_pred_sematic_label = []
    merged_pred_instance_label = []
    merged_gt_sematic_label = []
    merged_gt_instance_label = []

    # pre_file: "XXX_Result.ply"; gt_file: "XXX_Input.ply"
    pred_file_path = os.listdir(pre_folder_path)
    pred_file_path = [file_name for file_name in pred_file_path if file_name.endswith("_Result.ply")]
    all_file_names = [file_name.split("_")[0] for file_name in pred_file_path]

    for file_name in all_file_names:
        with open(os.path.join(pre_folder_path, file_name + "_Result.ply"), "rb") as f:
            plydata = PlyData.read(f)
            pre_sematic_label = plydata["vertex"]["predicted_semantic_label"]
            pre_instance_label = plydata["vertex"]["predicted_instance_label"]
            merged_pred_sematic_label.append(pre_sematic_label)
            merged_pred_instance_label.append(pre_instance_label)
            assert len(pre_sematic_label) == len(pre_instance_label)

        with open(os.path.join(gt_folder_path, file_name + "_Input.ply"), "rb") as f:
            plydata = PlyData.read(f)
            gt_sematic_label = plydata["vertex"]["semantic_label"]
            gt_instance_label = plydata["vertex"]["instance_label"]
            merged_gt_sematic_label.append(gt_sematic_label)
            merged_gt_instance_label.append(gt_instance_label)
            assert len(gt_sematic_label) == len(gt_instance_label)

    return merged_pred_sematic_label, merged_pred_instance_label, merged_gt_sematic_label, merged_gt_instance_label, all_file_names


def evaluate(pred_sematic_label, pred_instance_label, gt_sematic_label, gt_instance_label, plant_numbers) -> None:
    """
    :param pred_sematic_label:
    :param pred_instance_label:
    :param gt_sematic_label:
    :param gt_instance_label:
    :param plant_numbers:
    :return: None
    """
    # semantic evaluation
    semantic_result_dict = semantic_evaluation(pred_sematic_label, gt_sematic_label, plant_numbers)
    # instance evaluation
    instance_result_dict = instance_evaluation(pred_sematic_label, gt_sematic_label, pred_instance_label, gt_instance_label, plant_numbers)


def find_and_copy_ply_files(source_dir, destination_dir):
    # Ensure destination folder exists
    if not os.path.exists(destination_dir):
        os.makedirs(destination_dir)

    # Walk through the directory tree
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            # Check if the file ends with "_Input.ply"
            if file.endswith("_Result.ply"):
                file_path = os.path.join(root, file)
                dest_path = os.path.join(destination_dir, file)

                # Copy the file to the destination directory
                shutil.copy(file_path, dest_path)
                print(f"Copied: {file_path} to {dest_path}")


if __name__ == "__main__":
    source_directory = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/9_Result_10240"
    destination_directory = "../data/Output"
    find_and_copy_ply_files(source_directory, destination_directory)

    pred_sematic_label, pred_instance_label, gt_sematic_label, gt_instance_label, plant_numbers = get_total_pred_gt(
        "../data/Output", "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/All_Input_10240/PLY")
    evaluate(pred_sematic_label, pred_instance_label, gt_sematic_label, gt_instance_label, plant_numbers)
