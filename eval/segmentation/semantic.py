import os
import json

import numpy as np
from sklearn import metrics


def semantic_evaluation(pred, gt, plants, destination_directory) -> dict:
    """
    :param pred: semantic prediction, list of ndarray(N), binary classification
    :param gt: semantic ground truth, list of ndarray(N), binary classification
    :param plants: list of plant numbers
    :param destination_directory: str, path to save the results
    :return: dict, semantic evaluator results
    """
    results = {}

    # Evaluate each plant individually
    for i in range(len(plants)):
        pred_labels = pred[i]
        gt_labels = gt[i]

        # Calculate metrics for each class
        precision, recall, f1, _ = metrics.precision_recall_fscore_support(gt_labels, pred_labels, average=None,
                                                                           labels=[0, 1])
        precision_mean = np.mean(precision)
        recall_mean = np.mean(recall)
        f1_mean = np.mean(f1)

        # Calculate IoU for each class, stem: 0, leaf: 1
        iou_stem = metrics.jaccard_score(gt_labels, pred_labels, pos_label=0)
        iou_leaf = metrics.jaccard_score(gt_labels, pred_labels, pos_label=1)
        iou_mean = np.mean([iou_stem, iou_leaf])

        # Calculate overall accuracy
        accuracy = np.sum(pred_labels == gt_labels) / len(gt_labels)

        # Calculate per-class raw counts for micro-averaging
        tp_stem = int(np.sum((pred_labels == 0) & (gt_labels == 0)))
        fp_stem = int(np.sum((pred_labels == 0) & (gt_labels == 1)))
        fn_stem = int(np.sum((pred_labels == 1) & (gt_labels == 0)))
        tp_leaf = int(np.sum((pred_labels == 1) & (gt_labels == 1)))
        fp_leaf = fn_stem  # FP for leaf = FN for stem in binary
        fn_leaf = fp_stem  # FN for leaf = FP for stem in binary

        # Store the results in a dictionary
        single_plant_semantic_dict = {
            "Precision_stem": precision[0],
            "Precision_leaf": precision[1],
            "Precision_mean": precision_mean,
            "Recall_stem": recall[0],
            "Recall_leaf": recall[1],
            "Recall_mean": recall_mean,
            "F1_stem": f1[0],
            "F1_leaf": f1[1],
            "F1_mean": f1_mean,
            "IoU_stem": iou_stem,
            "IoU_leaf": iou_leaf,
            "IoU_mean": iou_mean,
            "Accuracy": accuracy,
            "_raw_counts": {
                "TP_stem": tp_stem,
                "FP_stem": fp_stem,
                "FN_stem": fn_stem,
                "TP_leaf": tp_leaf,
                "FP_leaf": fp_leaf,
                "FN_leaf": fn_leaf
            }
        }

        # Add the plant-specific metrics to the results dictionary
        results[f"{plants[i]}"] = single_plant_semantic_dict

    # Macro-average: mean of per-sample metrics
    metric_keys = [
        "Precision_stem", "Precision_leaf", "Precision_mean",
        "Recall_stem", "Recall_leaf", "Recall_mean",
        "F1_stem", "F1_leaf", "F1_mean",
        "IoU_stem", "IoU_leaf", "IoU_mean",
        "Accuracy",
    ]
    overall_semantic_dict = {}
    plant_results = [results[f"{plants[i]}"] for i in range(len(plants))]
    for key in metric_keys:
        vals = [r[key] for r in plant_results]
        overall_semantic_dict[key] = {
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0,
        }

    # Add the overall metrics to the results dictionary
    results["Overall"] = overall_semantic_dict

    # Saving the results to a JSON file
    try:
        with open(os.path.join(destination_directory, "semantic_evaluation_results_global.json"), "w") as json_file:
            json.dump(results, json_file, indent=4)
    except FileNotFoundError:
        print("Output directory not found. Please create it first.")

    return results


if __name__ == "__main__":
    pass
