#include "ground_segmentation.h"
/*
void LidarMapper::groundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
{
    // _ground_mat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (int j = 0; j < horizontal_scans_; j++)
    {
        for (int i = 0; i < ground_scan_idx_; i++)
        {
            int lower_idx = j + i * horizontal_scans_;
            int upper_idx = j + (i + 1) * horizontal_scans_;

            Point upper_pt = in_pts[upper_idx];
            Point lower_pt = in_pts[lower_idx];

            if (range_mat_(i, j) == -1 || range_mat_(i + 1, j) == -1)
            {
                // no info to check, invalid points
                //        ground_mat_(i, j) = -1;
                continue;
            }

            float dX = upper_pt.x_ - lower_pt.x_;
            float dY = upper_pt.y_ - lower_pt.y_;
            float dZ = upper_pt.z_ - lower_pt.z_;

            float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

            if (vertical_angle <= ground_th_)
            {
                out_pcl.points_.push_back(lower_pt);
                out_pcl.points_.push_back(upper_pt);
                out_pcl.indexes_.emplace_back(i, j);
                out_pcl.indexes_.emplace_back(i + 1, j);
            }
        }
    }
}

static bool ransac(const std::vector<Point>& in_pts, Plane& out_plane, int max_iters = 20, float dist_threshold = 0.08,
                   bool filter_distant_pts = false)
{
    std::vector<Point> pts;
    if (filter_distant_pts)
    {
        for (const auto& pt : in_pts)
        {
            if (pt.norm3D() < 5)
            {
                pts.push_back(pt);
            }
        }
    }
    else
    {
        pts = in_pts;
    }

    if (pts.empty())
    {
        return false;
    }

    int max_idx = static_cast<int>(pts.size()) - 1;
    int min_idx = 0;
    int max_tries = 1000;
    int c_max_inliers = 0;

    for (int i = 0; i < max_iters; i++)
    {
        // Declare private point cloud to store current solution
        std::vector<Point> l_pcl;

        // Reset number of inliers in each iteration
        int num_inliers = 0;

        // Randomly select three points that cannot be cohincident
        // TODO (André Aguiar): Also check if points are collinear
        bool found_valid_pts = false;
        int n = 0;
        int idx1, idx2, idx3;
        while (!found_valid_pts)
        {
            idx1 = std::rand() % (max_idx - min_idx + 1) + min_idx;
            idx2 = std::rand() % (max_idx - min_idx + 1) + min_idx;
            idx3 = std::rand() % (max_idx - min_idx + 1) + min_idx;

            if (idx1 != idx2 && idx1 != idx3 && idx2 != idx3)
                found_valid_pts = true;

            n++;
            if (n > max_tries)
                break;
        }

        if (!found_valid_pts)
        {
            std::cout << "WARNING (ransac): No valid set of points found ... " << std::endl;
            return false;
        }

        // Declare the 3 points selected on this iteration
        Point pt1 = Point(pts[idx1].x_, pts[idx1].y_, pts[idx1].z_);
        Point pt2 = Point(pts[idx2].x_, pts[idx2].y_, pts[idx2].z_);
        Point pt3 = Point(pts[idx3].x_, pts[idx3].y_, pts[idx3].z_);

        // Extract the plane hessian coefficients
        Vec v1(pt2, pt1);
        Vec v2(pt3, pt1);
        Vec abc = v1.cross(v2);
        float l_a = abc.x_;
        float l_b = abc.y_;
        float l_c = abc.z_;
        float l_d = -(l_a * pt1.x_ + l_b * pt1.y_ + l_c * pt1.z_);

        for (const auto& l_pt : pts)
        {
            // Compute the distance each point to the plane - from
            // https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
            auto norm = std::sqrt(l_a * l_a + l_b * l_b + l_c * l_c);
            if (std::fabs(l_a * l_pt.x_ + l_b * l_pt.y_ + l_c * l_pt.z_ + l_d) / norm < dist_threshold)
            {
                num_inliers++;
                l_pcl.push_back(l_pt);
            }
        }

        if (num_inliers > c_max_inliers)
        {
            c_max_inliers = num_inliers;

            out_plane.points_.clear();
            out_plane.points_ = l_pcl;
            out_plane.a_ = l_a;
            out_plane.b_ = l_b;
            out_plane.c_ = l_c;
            out_plane.d_ = l_d;
        }
    }

    // PCA-based normal refinement using all the inliers
    float l_a, l_b, l_c, l_d;
    estimateNormal(out_plane.points_, l_a, l_b, l_c, l_d);

    out_plane.a_ = l_a;
    out_plane.b_ = l_b;
    out_plane.c_ = l_c;
    out_plane.d_ = l_d;

    return c_max_inliers > 0;
}

*/