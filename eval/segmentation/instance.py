import os
import json

import numpy as np
from scipy import stats


def calculate_iou(ins_pred, ins_gt):
    union = np.sum(ins_pred | ins_gt)
    if union == 0:
        return 0.0
    intersect = np.sum(ins_pred & ins_gt)
    iou = intersect / union
    return iou


def instance_evaluation(pred_sem_label, gt_sem_label, pred_inst_label, gt_inst_label, plants, destination_directory):
    results = {}

    NUM_CLASSES = 2  # stem, leaf or just leaf

    total_gt_ins = np.zeros(NUM_CLASSES)
    iou_threshold = 0.5

    tpsins = [[] for _ in range(NUM_CLASSES)]
    fpsins = [[] for _ in range(NUM_CLASSES)]

    all_mean_cov = [[] for _ in range(NUM_CLASSES)]
    all_mean_weighted_cov = [[] for _ in range(NUM_CLASSES)]

    for plant_idx in range(len(plants)):
        plant_result = {
            "MUCov": np.zeros(NUM_CLASSES).tolist(),
            "MWCov": np.zeros(NUM_CLASSES).tolist(),
            "Precision": np.zeros(NUM_CLASSES).tolist(),
            "Recall": np.zeros(NUM_CLASSES).tolist(),
            "_raw_counts": {
                "tp": np.zeros(NUM_CLASSES, dtype=int).tolist(),
                "fp": np.zeros(NUM_CLASSES, dtype=int).tolist(),
                "num_gt": np.zeros(NUM_CLASSES, dtype=int).tolist()
            }
        }

        pred_ins = pred_inst_label[plant_idx]
        pred_sem = pred_sem_label[plant_idx]

        gt_ins = gt_inst_label[plant_idx]
        gt_sem = gt_sem_label[plant_idx]

        # Process predicted instances
        unique_pred_instances = np.unique(pred_ins)
        if len(unique_pred_instances) == 0:
            continue

        pts_in_pred = [[] for _ in range(NUM_CLASSES)]
        for instance_id in unique_pred_instances:
            if instance_id == -1:
                continue
            instance_mask = (pred_ins == instance_id)
            semantic_class = int(stats.mode(pred_sem[instance_mask])[0])  # Safe extraction
            if 0 <= semantic_class < NUM_CLASSES:
                pts_in_pred[semantic_class].append(instance_mask)

        # Process ground truth instances
        unique_gt_instances = np.unique(gt_ins)
        if len(unique_gt_instances) == 0:
            continue

        pts_in_gt = [[] for _ in range(NUM_CLASSES)]
        for instance_id in unique_gt_instances:
            instance_mask = (gt_ins == instance_id)
            semantic_class = int(stats.mode(gt_sem[instance_mask])[0])  # Safe extraction
            if 0 <= semantic_class < NUM_CLASSES:
                pts_in_gt[semantic_class].append(instance_mask)

        # Calculate MUCov and MWCov for the plant
        for i_sem in range(NUM_CLASSES):
            sum_cov = 0
            mean_weighted_cov = 0
            total_gt_points = 0
            for ins_gt in pts_in_gt[i_sem]:
                max_iou = 0.0
                num_gt_points = np.sum(ins_gt)
                total_gt_points += num_gt_points

                for ins_pred in pts_in_pred[i_sem]:
                    iou = calculate_iou(ins_pred, ins_gt)
                    max_iou = max(max_iou, iou)

                sum_cov += max_iou
                mean_weighted_cov += max_iou * num_gt_points

            if len(pts_in_gt[i_sem]) > 0:
                mean_cov = sum_cov / len(pts_in_gt[i_sem])
                all_mean_cov[i_sem].append(mean_cov)
                plant_result["MUCov"][i_sem] = mean_cov

                if total_gt_points > 0:
                    mean_weighted_cov /= total_gt_points
                all_mean_weighted_cov[i_sem].append(mean_weighted_cov)
                plant_result["MWCov"][i_sem] = mean_weighted_cov

        # Calculate Precision and Recall for the plant
        for i_sem in range(NUM_CLASSES):
            tp = [0] * len(pts_in_pred[i_sem])
            fp = [0] * len(pts_in_pred[i_sem])
            total_gt_ins[i_sem] += len(pts_in_gt[i_sem])

            matched_gt = set()
            for ip, ins_pred in enumerate(pts_in_pred[i_sem]):
                best_iou = -1
                best_gt_idx = -1
                for ig, ins_gt in enumerate(pts_in_gt[i_sem]):
                    if ig in matched_gt:
                        continue
                    iou = calculate_iou(ins_pred, ins_gt)
                    if iou > best_iou:
                        best_iou = iou
                        best_gt_idx = ig

                if best_iou >= iou_threshold:
                    tp[ip] = 1
                    matched_gt.add(best_gt_idx)
                else:
                    fp[ip] = 1

            tpsins[i_sem].extend(tp)
            fpsins[i_sem].extend(fp)

            tp_sum = int(np.sum(tp))
            fp_sum = int(np.sum(fp))
            num_gt = len(pts_in_gt[i_sem])

            plant_result["_raw_counts"]["tp"][i_sem] = tp_sum
            plant_result["_raw_counts"]["fp"][i_sem] = fp_sum
            plant_result["_raw_counts"]["num_gt"][i_sem] = num_gt

            if num_gt > 0:
                plant_result["Recall"][i_sem] = tp_sum / num_gt
            if (tp_sum + fp_sum) > 0:
                plant_result["Precision"][i_sem] = tp_sum / (tp_sum + fp_sum)
            else:
                plant_result["Precision"][i_sem] = 0.0

        results[f"{plants[plant_idx]}"] = plant_result

    all_precision = [[] for _ in range(NUM_CLASSES)]
    all_recall = [[] for _ in range(NUM_CLASSES)]
    for plant_name in results:
        for i_sem in range(NUM_CLASSES):
            all_precision[i_sem].append(results[plant_name]["Precision"][i_sem])
            all_recall[i_sem].append(results[plant_name]["Recall"][i_sem])

    overall_results = {}
    for metric_name, data_lists in [
        ("MUCov", all_mean_cov),
        ("MWCov", all_mean_weighted_cov),
        ("Precision", all_precision),
        ("Recall", all_recall),
    ]:
        mean_per_class = []
        std_per_class = []
        for i_sem in range(NUM_CLASSES):
            vals = data_lists[i_sem]
            mean_per_class.append(float(np.mean(vals)) if vals else 0.0)
            std_per_class.append(float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)
        overall_results[metric_name] = {"mean": mean_per_class, "std": std_per_class}

    results["Overall"] = overall_results

    # Save results as a JSON file
    try:
        with open(os.path.join(destination_directory, "instance_evaluation_results_global.json"), "w") as json_file:
            json.dump(results, json_file, indent=4)
    except FileNotFoundError:
        print("Output directory not found. Please create it first.")

    return results


if __name__ == "__main__":
    pass
