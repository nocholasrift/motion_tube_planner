#include <motion_tube_planner/template_generator.h>

#include <iostream>

void TemplateGenerator::generateTemplate(const Eigen::Vector2d& input, double horizon)
{
    double w = input[1];

    Template& t = templates_.emplace_back();

    if (fabs(w) < 1e-3)
    {
        // straight line tube
        Eigen::Vector2d left_pt  = footprint_[0] + Eigen::Vector2d(0, d_aug_);
        Eigen::Vector2d right_pt = footprint_[1] - Eigen::Vector2d(0, d_aug_);

        t.left_bdry  = projectPt(left_pt, input, horizon);
        t.right_bdry = projectPt(right_pt, input, horizon);
        t.top_bdry   = interpolate({t.left_bdry.back(), t.right_bdry.back()});
    }
    else if (w > 0)
    {
        Eigen::Vector2d left_pt  = footprint_[3] + Eigen::Vector2d(0, d_aug_);
        Eigen::Vector2d right_pt = footprint_[1] - Eigen::Vector2d(0, d_aug_);

        // left turn tube
        t.left_bdry = projectPt(left_pt, input, horizon);

        Eigen::Vector2d fl_projected       = projectPt(footprint_[0], input, horizon).back();
        std::vector<Eigen::Vector2d> bl2fl = interpolate({t.left_bdry.back(), fl_projected});
        t.left_bdry.insert(t.left_bdry.end(), bl2fl.begin(), bl2fl.end());

        t.right_bdry = projectPt(right_pt, input, horizon);
        t.top_bdry   = interpolate({t.left_bdry.back(), t.right_bdry.back()});
    }
    else
    {
        Eigen::Vector2d left_pt  = footprint_[0] + Eigen::Vector2d(0, d_aug_);
        Eigen::Vector2d right_pt = footprint_[2] - Eigen::Vector2d(0, d_aug_);

        // right turn tube
        t.left_bdry  = projectPt(left_pt, input, horizon);
        t.right_bdry = projectPt(right_pt, input, horizon);

        Eigen::Vector2d fr_projected       = projectPt(footprint_[1], input, horizon).back();
        std::vector<Eigen::Vector2d> br2fr = interpolate({t.right_bdry.back(), fr_projected});
        t.right_bdry.insert(t.right_bdry.end(), br2fr.begin(), br2fr.end());

        t.top_bdry = interpolate({t.left_bdry.back(), t.right_bdry.back()});
    }

    // convert to indices
    std::vector<Eigen::Vector2d> bdry_pts;
    bdry_pts.insert(bdry_pts.end(), t.left_bdry.begin(), t.left_bdry.end());
    bdry_pts.insert(bdry_pts.end(), t.top_bdry.begin(), t.top_bdry.end());
    bdry_pts.insert(bdry_pts.end(), t.right_bdry.rbegin(), t.right_bdry.rend());

    t.all_pts = bdry_pts;
    t.inds    = convertTemplateToInds(bdry_pts);
    t.input   = input;
}

std::vector<Eigen::Vector2d> TemplateGenerator::projectPt(const Eigen::Vector2d& pt,
                                                          const Eigen::Vector2d& input,
                                                          double horizon)
{
    // compute sampling time from sampling distance
    double v = input[0];
    double w = input[1];

    double dt = d_sample_ / v;

    if (fabs(w) >= 1e-3)
    {
        double yterm = v / fabs(w) - pt[1];
        double rp    = sqrt(pt[0] * pt[0] + yterm * yterm);
        dt           = d_sample_ / (fabs(w) * rp);
    }

    int n = static_cast<int>(horizon / dt) + 1;
    dt    = horizon / (n - 1);

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(n);
    for (double t = 0; t < horizon; t += dt)
    {
        Eigen::Vector2d& projected_pt = pts.emplace_back();
        if (fabs(w) < 1e-3)
        {
            projected_pt[0] = pt[0] + v * t;
            projected_pt[1] = pt[1];
        }
        else
        {
            projected_pt[0] = pt[0] * cos(w * t) - pt[1] * sin(w * t) + v / w * sin(w * t);
            projected_pt[1] =
                pt[0] * sin(w * t) + pt[1] * cos(w * t) + v / w * (1 - cos(w * t));
        }
    }

    return pts;
}

std::vector<Eigen::Vector2d> TemplateGenerator::interpolate(
    const std::vector<Eigen::Vector2d>& pts)
{
    // get interior line segment points at d_sample intervals
    std::vector<Eigen::Vector2d> interpolated_pts;
    if (pts.size() != 2)
    {
        std::cout << "[TemplateGenerator::Interpolate] Expected exactly 2 input points, got "
                  << pts.size() << std::endl;
    }

    Eigen::Vector2d dir = pts[1] - pts[0];
    double len          = dir.norm();
    dir.normalize();

    unsigned int n = static_cast<unsigned int>(len / d_sample_);
    interpolated_pts.reserve(n);

    for (double d = d_sample_; d < len; d += d_sample_)
    {
        Eigen::Vector2d pt = pts[0] + dir * d;
        interpolated_pts.emplace_back(pt);
    }

    return interpolated_pts;
}

std::vector<unsigned int> TemplateGenerator::convertTemplateToInds(
    const std::vector<Eigen::Vector2d>& template_pts)
{
    std::vector<unsigned int> inds;
    inds.reserve(template_pts.size());
    for (int i = 0; i < template_pts.size(); ++i)
    {
        const Eigen::Vector2d& pt = template_pts[i];
        double theta              = atan2(pt[1], pt[0]);

        if (theta < lidar_metadata_.angle_min || theta > lidar_metadata_.angle_max) return {};

        unsigned int ind = static_cast<unsigned int>((theta - lidar_metadata_.angle_min) /
                                                     lidar_metadata_.angle_increment);
        inds.emplace_back(ind);
    }

    return inds;
}
