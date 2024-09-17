import numpy as np
import json
from sklearn import metrics


def semantic_evaluation(pred, gt, plants) -> dict:
    """
    :param pred: semantic prediction, list of ndarray(N), binary classification
    :param gt: semantic ground truth, list of ndarray(N), binary classification
    :param plants: list of plant numbers
    :return: dict, semantic evaluation results
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
            "IoU_mean": iou_mean
        }

        # Add the plant-specific metrics to the results dictionary
        results[f"{plants[i]}"] = single_plant_semantic_dict

    # Concatenate all the pred and gt from all plants, transform the list of arrays into a single array
    all_pred_labels = np.concatenate(pred)
    all_gt_labels = np.concatenate(gt)

    # Calculate overall metrics
    precision, recall, f1, _ = metrics.precision_recall_fscore_support(all_gt_labels, all_pred_labels, average=None,
                                                                       labels=[0, 1])
    precision_mean = np.mean(precision)
    recall_mean = np.mean(recall)
    f1_mean = np.mean(f1)

    iou_stem = metrics.jaccard_score(all_gt_labels, all_pred_labels, pos_label=0)
    iou_leaf = metrics.jaccard_score(all_gt_labels, all_pred_labels, pos_label=1)
    iou_mean = np.mean([iou_stem, iou_leaf])

    # Store the overall results in a dictionary
    overall_semantic_dict = {
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
        "IoU_mean": iou_mean
    }

    # Add the overall metrics to the results dictionary
    results["Overall"] = overall_semantic_dict

    # Saving the results to a JSON file
    with open("../data/Output/semantic_evaluation_results_global.json", "w") as f:
        json.dump(results, f, indent=4)

    return results


if __name__ == "__main__":
    pass
