#pragma once
#include <Eigen/Core>

struct LidarMetadata
{
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
};

struct Template
{
    std::vector<Eigen::Vector2d> left_bdry;
    std::vector<Eigen::Vector2d> right_bdry;
    std::vector<Eigen::Vector2d> top_bdry;
    std::vector<Eigen::Vector2d> all_pts;

    std::vector<unsigned int> inds;

    double horizon;
    Eigen::Vector2d input;
};

class TemplateGenerator
{
   public:
    TemplateGenerator(const std::vector<Eigen::Vector2d>& footprint,
                      const LidarMetadata& meta_data, double d_aug, double d_sample)
        : footprint_(footprint), lidar_metadata_(meta_data), d_aug_(d_aug), d_sample_(d_sample)
    {
    }

    void generateTemplate(const Eigen::Vector2d& input, double horizon);

    std::vector<Eigen::Vector2d> projectPt(const Eigen::Vector2d& pt,
                                           const Eigen::Vector2d& input, double horizon);

    std::vector<Eigen::Vector2d> interpolate(const std::vector<Eigen::Vector2d>& pts);

    std::vector<unsigned int> convertTemplateToInds(
        const std::vector<Eigen::Vector2d>& template_pts);

    std::vector<Template> templates_;

   private:
    std::vector<Eigen::Vector2d> footprint_;
    LidarMetadata lidar_metadata_;

    double d_aug_;
    double d_sample_;
};
