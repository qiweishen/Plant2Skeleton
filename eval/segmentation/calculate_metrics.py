import json
import statistics
import sys


def calculate_instance_overall(data, sample_names):
    """Calculate Overall for instance evaluation metrics.
    Macro-average: mean ± std of per-sample values, reported per class.
    Class index 0 = stem, 1 = leaf.
    """
    metrics = ["MUCov", "MWCov", "Precision", "Recall"]
    num_classes = len(data[sample_names[0]][metrics[0]])
    overall = {}
    for metric in metrics:
        mean_per_class = []
        std_per_class = []
        for i_sem in range(num_classes):
            values = [data[name][metric][i_sem] for name in sample_names]
            mean_per_class.append(sum(values) / len(values))
            std_per_class.append(statistics.stdev(values) if len(values) > 1 else 0.0)
        overall[metric] = {"mean": mean_per_class, "std": std_per_class}
    return overall


def calculate_semantic_overall(data, sample_names):
    """Calculate Overall for semantic evaluation metrics.
    Macro-average: mean ± std of per-sample values.
    """
    metrics = [
        "Precision_stem", "Precision_leaf", "Precision_mean",
        "Recall_stem", "Recall_leaf", "Recall_mean",
        "F1_stem", "F1_leaf", "F1_mean",
        "IoU_stem", "IoU_leaf", "IoU_mean",
    ]
    overall = {}
    for metric in metrics:
        values = [data[name][metric] for name in sample_names]
        mean = sum(values) / len(values)
        std = statistics.stdev(values) if len(values) > 1 else 0.0
        overall[metric] = {"mean": mean, "std": std}
    return overall


def main():
    # User-provided sample names
    sample_names = []

    # Override with command-line arguments if provided
    if len(sys.argv) > 1:
        sample_names = sys.argv[1:]

    # Load data
    with open("/workspace/data/All/evaluate/instance_evaluation_results_global.json") as f:
        instance_data = json.load(f)
    with open("/workspace/data/All/evaluate/semantic_evaluation_results_global.json") as f:
        semantic_data = json.load(f)

    # Validate sample names
    missing_instance = [n for n in sample_names if n not in instance_data]
    missing_semantic = [n for n in sample_names if n not in semantic_data]
    if missing_instance:
        print(f"Warning: samples not found in instance data: {missing_instance}")
    if missing_semantic:
        print(f"Warning: samples not found in semantic data: {missing_semantic}")

    valid_instance = [n for n in sample_names if n in instance_data]
    valid_semantic = [n for n in sample_names if n in semantic_data]

    if not valid_instance and not valid_semantic:
        print("Error: no valid sample names provided.")
        sys.exit(1)

    print(f"Calculating Overall from {len(valid_instance)} instance / {len(valid_semantic)} semantic samples\n")

    # Calculate and print instance Overall
    if valid_instance:
        inst_overall = calculate_instance_overall(instance_data, valid_instance)
        print("=== Instance Evaluation Overall ===")
        print(json.dumps({"Overall": inst_overall}, indent=4))

    print()

    # Calculate and print semantic Overall
    if valid_semantic:
        sem_overall = calculate_semantic_overall(semantic_data, valid_semantic)
        print("=== Semantic Evaluation Overall ===")
        print(json.dumps({"Overall": sem_overall}, indent=4))


if __name__ == "__main__":
    main()
