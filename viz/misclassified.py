import os

import numpy as np
from scipy import stats
from plyfile import PlyData, PlyElement


def calculate_iou(ins_pred, ins_gt):
    union = np.sum(ins_pred | ins_gt)
    if union == 0:
        return 0.0
    return np.sum(ins_pred & ins_gt) / union


def match_instances(pred_ins, pred_sem, gt_ins, gt_sem, num_classes=2, iou_threshold=0.5):
    """
    Greedy one-to-one matching of predicted instances to GT instances.
    :return: dict mapping pred_instance_id -> gt_instance_id
    """
    pts_in_pred = [[] for _ in range(num_classes)]
    pred_ids = [[] for _ in range(num_classes)]
    for inst_id in np.unique(pred_ins):
        if inst_id == -1:
            continue
        mask = (pred_ins == inst_id)
        sem_class = int(stats.mode(pred_sem[mask])[0])
        if 0 <= sem_class < num_classes:
            pts_in_pred[sem_class].append(mask)
            pred_ids[sem_class].append(inst_id)

    pts_in_gt = [[] for _ in range(num_classes)]
    gt_ids = [[] for _ in range(num_classes)]
    for inst_id in np.unique(gt_ins):
        mask = (gt_ins == inst_id)
        sem_class = int(stats.mode(gt_sem[mask])[0])
        if 0 <= sem_class < num_classes:
            pts_in_gt[sem_class].append(mask)
            gt_ids[sem_class].append(inst_id)

    mapping = {}
    for i_sem in range(num_classes):
        matched_gt = set()
        for ip, pred_mask in enumerate(pts_in_pred[i_sem]):
            best_iou = -1
            best_gt_idx = -1
            for ig, gt_mask in enumerate(pts_in_gt[i_sem]):
                if ig in matched_gt:
                    continue
                iou = calculate_iou(pred_mask, gt_mask)
                if iou > best_iou:
                    best_iou = iou
                    best_gt_idx = ig
            if best_iou >= iou_threshold:
                mapping[pred_ids[i_sem][ip]] = gt_ids[i_sem][best_gt_idx]
                matched_gt.add(best_gt_idx)

    return mapping


def _write_eval_ply(output_file, x, y, z, eval_label, label_name):
    """Write a single evaluation PLY with RGB coloring: 1=(70,208,93), 0=(197,92,77)."""
    N = len(x)
    red = np.where(eval_label == 1, 70, 197).astype(np.uint8)
    green = np.where(eval_label == 1, 208, 92).astype(np.uint8)
    blue = np.where(eval_label == 1, 93, 77).astype(np.uint8)

    vertex = np.zeros(N, dtype=[
        ('x', 'f8'), ('y', 'f8'), ('z', 'f8'),
        (label_name, 'i4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
    ])
    vertex['x'] = x
    vertex['y'] = y
    vertex['z'] = z
    vertex[label_name] = eval_label
    vertex['red'] = red
    vertex['green'] = green
    vertex['blue'] = blue

    PlyData([PlyElement.describe(vertex, 'vertex')], text=False).write(output_file)
    print(f"Saved: {output_file}")


def generate_misclassified_ply(pred_file, gt_file, output_sem_file, output_ins_file, iou_threshold=0.5):
    """
    Generate two PLY files marking point-level correctness with RGB.
    - Semantic PLY: eval-semantic 1(correct)/0(wrong), green/red
    - Instance PLY: eval-instance 1(correct)/0(wrong), green/red
    """
    pred_ply = PlyData.read(pred_file)
    gt_ply = PlyData.read(gt_file)

    x = np.array(pred_ply["vertex"]["x"])
    y = np.array(pred_ply["vertex"]["y"])
    z = np.array(pred_ply["vertex"]["z"])
    pred_sem = np.array(pred_ply["vertex"]["pred-semantic"])
    pred_ins = np.array(pred_ply["vertex"]["pred-instance"])
    gt_sem = np.array(gt_ply["vertex"]["semantic"])
    gt_ins = np.array(gt_ply["vertex"]["instance"])

    N = len(x)

    # Semantic: 1 = correct, 0 = misclassified
    eval_semantic = (pred_sem == gt_sem).astype(np.int32)

    # Instance: 1 = correct, 0 = misclassified (leaf only; stem points default to 1)
    mapping = match_instances(pred_ins, pred_sem, gt_ins, gt_sem, iou_threshold=iou_threshold)

    mapped_gt = np.full(N, -999, dtype=np.int32)
    for pred_id, gt_id in mapping.items():
        mapped_gt[pred_ins == pred_id] = gt_id

    eval_instance = (mapped_gt == gt_ins).astype(np.int32)
    # Ignore stem points and stem-leaf misclassifications; only show leaf-to-leaf instance errors
    eval_instance[(gt_sem == 0) | (pred_sem != gt_sem)] = 1

    # Write PLY files (all points)
    _write_eval_ply(output_sem_file, x, y, z, eval_semantic, 'eval-semantic')
    _write_eval_ply(output_ins_file, x, y, z, eval_instance, 'eval-instance')


def generate_all(pred_folder, gt_folder, output_folder, iou_threshold=0.5):
    """Process all plants: XXX_Result.ply + XXX_Input.ply -> XXX_Eval_Semantic.ply + XXX_Eval_Instance.ply"""
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    pred_files = [f for f in os.listdir(pred_folder) if f.endswith("_Result.ply")]
    for pred_file in pred_files:
        plant_name = pred_file.split("_")[0]
        gt_path = os.path.join(gt_folder, plant_name + "_Input.ply")

        if not os.path.exists(gt_path):
            print(f"GT not found for {plant_name}, skipping.")
            continue

        generate_misclassified_ply(
            os.path.join(pred_folder, pred_file),
            gt_path,
            os.path.join(output_folder, plant_name + "_Eval_Semantic.ply"),
            os.path.join(output_folder, plant_name + "_Eval_Instance.ply"),
            iou_threshold
        )


if __name__ == "__main__":
    data_dir = "/data/Examples/evaluate"
    output_dir = "/data/Examples/evaluate"
    generate_all(data_dir, data_dir, output_dir)
