PointCloud::VoxelDownSampleAndTrace(double voxel_size,
                                    const Eigen::Vector3d &min_bound,
                                    const Eigen::Vector3d &max_bound,
                                    bool approximate_class) const {
    auto output = std::make_shared<PointCloud>();
    Eigen::MatrixXi cubic_id;
    if (voxel_size <= 0.0) {
        utility::LogError("voxel_size <= 0.");
    }
    // Note: this is different from VoxelDownSample.
    // It is for fixing coordinate for multiscale voxel space
    auto voxel_min_bound = min_bound;
    auto voxel_max_bound = max_bound;
    if (voxel_size * std::numeric_limits<int>::max() <
        (voxel_max_bound - voxel_min_bound).maxCoeff()) {
        utility::LogError("voxel_size is too small.");
    }
    std::unordered_map<Eigen::Vector3i, AccumulatedPointForTrace,
    utility::hash_eigen<Eigen::Vector3i>>
            voxelindex_to_accpoint;
    int cid_temp[3] = {1, 2, 4};
    for (size_t i = 0; i < points_.size(); i++) {
        auto ref_coord = (points_[i] - voxel_min_bound) / voxel_size;
        auto voxel_index = Eigen::Vector3i(int(floor(ref_coord(0))),
                                           int(floor(ref_coord(1))),
                                           int(floor(ref_coord(2))));
        int cid = 0;
        for (int c = 0; c < 3; c++) {
            if ((ref_coord(c) - voxel_index(c)) >= 0.5) {
                cid += cid_temp[c];
            }
        }
        voxelindex_to_accpoint[voxel_index].AddPoint(*this, i, cid,
                                                     approximate_class);
    }
    bool has_normals = HasNormals();
    bool has_colors = HasColors();
    bool has_covariances = HasCovariances();
    int cnt = 0;
    cubic_id.resize(voxelindex_to_accpoint.size(), 8);
    cubic_id.setConstant(-1);
    std::vector<std::vector<int>> original_indices(
            voxelindex_to_accpoint.size());
    for (auto accpoint : voxelindex_to_accpoint) {
        output->points_.push_back(accpoint.second.GetAveragePoint());
        if (has_normals) {
            output->normals_.push_back(accpoint.second.GetAverageNormal());
        }
        if (has_colors) {
            if (approximate_class) {
                output->colors_.push_back(accpoint.second.GetMaxClass());
            } else {
                output->colors_.push_back(accpoint.second.GetAverageColor());
            }
        }
        if (has_covariances) {
            output->covariances_.emplace_back(
                    accpoint.second.GetAverageCovariance());
        }
        auto original_id = accpoint.second.GetOriginalID();
        for (int i = 0; i < (int)original_id.size(); i++) {
            size_t pid = original_id[i].point_id;
            int cid = original_id[i].cubic_id;
            cubic_id(cnt, cid) = int(pid);
            original_indices[cnt].push_back(int(pid));
        }
        cnt++;
    }
    utility::LogDebug(
            "Pointcloud down sampled from {:d} points to {:d} points.",
            (int)points_.size(), (int)output->points_.size());
    return std::make_tuple(output, cubic_id, original_indices);
}