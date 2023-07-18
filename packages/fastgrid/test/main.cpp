#include <gtest/gtest.h>

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "fastgrid/manhattan_distance.h"
#include "fastgrid/distance_transform.h"
#include "fastgrid/interpolation.h"
#include "geom/pose.h"

#include <cmath>
#include <limits>
#include <memory>
#include <opencv2/imgproc.hpp>

using namespace truck::geom;
using namespace truck::fastgrid;

TEST(Size, operability) {
    const Size a = {.width = 1, .height = 2};
    EXPECT_EQ(a.width, 1);
    EXPECT_EQ(a.height, 2);
    EXPECT_EQ(a(), 2);
}

TEST(Grid, operability) {
    const Size sz_1 = {.width = 2, .height = 3};
    const int res_1 = 10;
    const Pose pose_1({1, 2}, {3, 4});
    std::unique_ptr<int[]> data_1(new int[6]);
    data_1[1] = 2;

    Grid<int> grid_1(sz_1, res_1, pose_1);
    grid_1.Reset(data_1.get());
    EXPECT_EQ(grid_1.size.width, 2);
    EXPECT_EQ(grid_1.size.height, 3);
    EXPECT_EQ(grid_1.resolution, 10);
    EXPECT_EQ(grid_1.data[1], 2);
    EXPECT_EQ(grid_1.origin->pos.x, 1);
    EXPECT_EQ(grid_1.origin->pos.y, 2);
    EXPECT_EQ(grid_1.origin->dir.x, 3);
    EXPECT_EQ(grid_1.origin->dir.y, 4);

    const Size sz_2 = {.width = 2, .height = 3};
    const int res_2 = 10;

    U8Grid grid_2(sz_2, res_2);
    EXPECT_EQ(grid_2.size.width, 2);
    EXPECT_EQ(grid_2.size.height, 3);
    EXPECT_EQ(grid_2.resolution, 10);
    EXPECT_EQ(grid_2.origin, std::nullopt);
}

TEST(GridDataPtr, allocation_and_workabillity) {
    const Size sz = {.width = 2, .height = 3};

    GridDataPtr<int> ptr_1 = Allocate<int>(sz);
    ptr_1[5] = 2;
    EXPECT_EQ(ptr_1[5], 2);

    U8GridDataPtr ptr_2 = Allocate<uint8_t>(sz);
    ptr_2[5] = 2;
    EXPECT_EQ(ptr_2[5], 2);
}

TEST(GridHolder, make_grid) {
    const Size sz_1 = {.width = 2, .height = 3};
    const int res_1 = 10;
    const Pose pose_1({1, 2}, {3, 4});
    S32GridDataPtr data_1 = Allocate<int32_t>(sz_1);
    data_1[1] = 2;

    Grid<int> grid_1(sz_1, res_1, pose_1);
    grid_1.Reset(data_1.get());

    GridHolder<int> grid_holder_1 = MakeGridLike<int>(grid_1);
    EXPECT_EQ(grid_holder_1.grid.size.width, 2);
    EXPECT_EQ(grid_holder_1.grid.size.height, 3);
    EXPECT_EQ(grid_holder_1.grid.resolution, 10);
    EXPECT_EQ(grid_holder_1.grid.data[1], 2);
    EXPECT_EQ(grid_holder_1.grid.origin->pos.x, 1);
    EXPECT_EQ(grid_holder_1.grid.origin->pos.y, 2);
    EXPECT_EQ(grid_holder_1.grid.origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_1).origin->dir.y, 4);
    EXPECT_EQ((*grid_holder_1).size.width, 2);
    EXPECT_EQ((*grid_holder_1).size.height, 3);
    EXPECT_EQ((*grid_holder_1).resolution, 10);
    EXPECT_EQ((*grid_holder_1).data[1], 2);
    EXPECT_EQ((*grid_holder_1).origin->pos.x, 1);
    EXPECT_EQ((*grid_holder_1).origin->pos.y, 2);
    EXPECT_EQ((*grid_holder_1).origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_1).origin->dir.y, 4);
    EXPECT_EQ(grid_holder_1->size.width, 2);
    EXPECT_EQ(grid_holder_1->size.height, 3);
    EXPECT_EQ(grid_holder_1->resolution, 10);
    EXPECT_EQ(grid_holder_1->data[1], 2);
    EXPECT_EQ(grid_holder_1->origin->pos.x, 1);
    EXPECT_EQ(grid_holder_1->origin->pos.y, 2);
    EXPECT_EQ(grid_holder_1->origin->dir.x, 3);

    GridHolder<int> grid_holder_2(MakeGridLike<int>(grid_holder_1));
    EXPECT_EQ(grid_holder_2.grid.size.width, 2);
    EXPECT_EQ(grid_holder_2.grid.size.height, 3);
    EXPECT_EQ(grid_holder_2.grid.resolution, 10);
    EXPECT_EQ(grid_holder_2.grid.data[1], 2);
    EXPECT_EQ(grid_holder_2.grid.origin->pos.x, 1);
    EXPECT_EQ(grid_holder_2.grid.origin->pos.y, 2);
    EXPECT_EQ(grid_holder_2.grid.origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_2).origin->dir.y, 4);
    EXPECT_EQ((*grid_holder_2).size.width, 2);
    EXPECT_EQ((*grid_holder_2).size.height, 3);
    EXPECT_EQ((*grid_holder_2).resolution, 10);
    EXPECT_EQ((*grid_holder_2).data[1], 2);
    EXPECT_EQ((*grid_holder_2).origin->pos.x, 1);
    EXPECT_EQ((*grid_holder_2).origin->pos.y, 2);
    EXPECT_EQ((*grid_holder_2).origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_2).origin->dir.y, 4);
    EXPECT_EQ(grid_holder_2->size.width, 2);
    EXPECT_EQ(grid_holder_2->size.height, 3);
    EXPECT_EQ(grid_holder_2->resolution, 10);
    EXPECT_EQ(grid_holder_2->data[1], 2);
    EXPECT_EQ(grid_holder_2->origin->pos.x, 1);
    EXPECT_EQ(grid_holder_2->origin->pos.y, 2);
    EXPECT_EQ(grid_holder_2->origin->dir.x, 3);
}

TEST(ManhattanDistance, operability) {
    const float unreachable = std::numeric_limits<float>::max();

    const Size sz_1 = {.width = 2, .height = 3};
    const int res_1 = 1;
    const Pose pose_1({1, 2}, {3, 4});
    F32GridDataPtr data_1 = Allocate<float>(sz_1);

    F32Grid grid_1(sz_1, res_1, pose_1);
    grid_1.Reset(data_1.get());

    grid_1[0][0] = 0;
    grid_1[0][1] = 1;
    grid_1[1][0] = 1;
    grid_1[1][1] = sqrt(2);
    grid_1[2][0] = 2;
    grid_1[2][1] = sqrt(5);

    F32GridHolder holder_1 = ManhattanDistance(grid_1, {1.5, 2.5}, 0.5);

    EXPECT_EQ(holder_1.grid[0][0], unreachable);
    EXPECT_EQ(holder_1.grid[0][1], 2);
    EXPECT_EQ(holder_1.grid[1][0], 2);
    EXPECT_EQ(holder_1.grid[1][1], 1);
    EXPECT_EQ(holder_1.grid[2][0], 1);
    EXPECT_EQ(holder_1.grid[2][1], 0);

    const Size sz_2 = {.width = 2, .height = 3};
    const int res_2 = 1;
    const Pose pose_2({1, 2}, {3, 4});
    F32GridDataPtr data_2 = Allocate<float>(sz_2);

    F32Grid grid_2(sz_2, res_2, pose_2);
    grid_2.Reset(data_2.get());

    grid_2[0][0] = 0;
    grid_2[0][1] = 1;
    grid_2[1][0] = 1;
    grid_2[1][1] = sqrt(2);
    grid_2[2][0] = 2;
    grid_2[2][1] = sqrt(5);

    F32GridHolder holder_2 = ManhattanDistance(grid_2, {1.5, 2.5}, 1.5);

    EXPECT_EQ(holder_2.grid[0][0], unreachable);
    EXPECT_EQ(holder_2.grid[0][1], unreachable);
    EXPECT_EQ(holder_2.grid[1][0], unreachable);
    EXPECT_EQ(holder_2.grid[1][1], unreachable);
    EXPECT_EQ(holder_2.grid[2][0], 1);
    EXPECT_EQ(holder_2.grid[2][1], 0);

    const Size sz_3 = {.width = 5, .height = 5};
    const int res_3 = 1;
    const Pose pose_3({1, 2}, {3, 4});
    F32GridDataPtr data_3 = Allocate<float>(sz_3);

    F32Grid grid_3(sz_3, res_3, pose_3);
    grid_3.Reset(data_3.get());

    grid_3[0][0] = sqrt(5);
    grid_3[0][1] = sqrt(2);
    grid_3[0][2] = 1;
    grid_3[0][3] = 1;
    grid_3[0][4] = sqrt(2);
    grid_3[1][0] = sqrt(2);
    grid_3[1][1] = 1;
    grid_3[1][2] = 0;
    grid_3[1][3] = 0;
    grid_3[1][4] = 1;
    grid_3[2][0] = 1;
    grid_3[2][1] = 0;
    grid_3[2][2] = 1;
    grid_3[2][3] = 1;
    grid_3[2][4] = sqrt(2);
    grid_3[3][0] = 1;
    grid_3[3][1] = 0;
    grid_3[3][2] = 1;
    grid_3[3][3] = 0;
    grid_3[3][4] = 1;
    grid_3[4][0] = sqrt(2);
    grid_3[4][1] = 1;
    grid_3[4][2] = 1;
    grid_3[4][3] = 0;
    grid_3[4][4] = 1;

    F32GridHolder holder_3 = ManhattanDistance(grid_3, {2, 2}, 0.5);

    EXPECT_EQ(holder_3.grid[0][0], 8);
    EXPECT_EQ(holder_3.grid[0][1], 7);
    EXPECT_EQ(holder_3.grid[0][2], 6);
    EXPECT_EQ(holder_3.grid[0][3], 5);
    EXPECT_EQ(holder_3.grid[0][4], 4);
    EXPECT_EQ(holder_3.grid[1][0], 7);
    EXPECT_EQ(holder_3.grid[1][1], 8);
    EXPECT_EQ(holder_3.grid[1][2], unreachable);
    EXPECT_EQ(holder_3.grid[1][3], unreachable);
    EXPECT_EQ(holder_3.grid[1][4], 3);
    EXPECT_EQ(holder_3.grid[2][0], 6);
    EXPECT_EQ(holder_3.grid[2][1], unreachable);
    EXPECT_EQ(holder_3.grid[2][2], 0);
    EXPECT_EQ(holder_3.grid[2][3], 1);
    EXPECT_EQ(holder_3.grid[2][4], 2);
    EXPECT_EQ(holder_3.grid[3][0], 5);
    EXPECT_EQ(holder_3.grid[3][1], unreachable);
    EXPECT_EQ(holder_3.grid[3][2], 1);
    EXPECT_EQ(holder_3.grid[3][3], unreachable);
    EXPECT_EQ(holder_3.grid[3][4], 3);
    EXPECT_EQ(holder_3.grid[4][0], 4);
    EXPECT_EQ(holder_3.grid[4][1], 3);
    EXPECT_EQ(holder_3.grid[4][2], 2);
    EXPECT_EQ(holder_3.grid[4][3], unreachable);
    EXPECT_EQ(holder_3.grid[4][4], 4);

    const Size sz_4 = {.width = 5, .height = 5};
    const int res_4 = 1;
    const Pose pose_4({1, 2}, {3, 4});
    F32GridDataPtr data_4 = Allocate<float>(sz_4);

    F32Grid grid_4(sz_4, res_4, pose_4);
    grid_4.Reset(data_4.get());

    grid_4[0][0] = sqrt(5);
    grid_4[0][1] = sqrt(2);
    grid_4[0][2] = 1;
    grid_4[0][3] = 1;
    grid_4[0][4] = sqrt(2);
    grid_4[1][0] = sqrt(2);
    grid_4[1][1] = 1;
    grid_4[1][2] = 0;
    grid_4[1][3] = 0;
    grid_4[1][4] = 1;
    grid_4[2][0] = 1;
    grid_4[2][1] = 0;
    grid_4[2][2] = 1;
    grid_4[2][3] = 1;
    grid_4[2][4] = sqrt(2);
    grid_4[3][0] = 1;
    grid_4[3][1] = 0;
    grid_4[3][2] = 1;
    grid_4[3][3] = 0;
    grid_4[3][4] = 1;
    grid_4[4][0] = sqrt(2);
    grid_4[4][1] = 1;
    grid_4[4][2] = 1;
    grid_4[4][3] = 0;
    grid_4[4][4] = 1;

    F32GridHolder holder_4 = ManhattanDistance(grid_4, {2, 1}, 0.5);

    EXPECT_EQ(holder_4.grid[0][0], unreachable);
    EXPECT_EQ(holder_4.grid[0][1], unreachable);
    EXPECT_EQ(holder_4.grid[0][2], unreachable);
    EXPECT_EQ(holder_4.grid[0][3], unreachable);
    EXPECT_EQ(holder_4.grid[0][4], unreachable);
    EXPECT_EQ(holder_4.grid[1][0], unreachable);
    EXPECT_EQ(holder_4.grid[1][1], unreachable);
    EXPECT_EQ(holder_4.grid[1][2], unreachable);
    EXPECT_EQ(holder_4.grid[1][3], unreachable);
    EXPECT_EQ(holder_4.grid[1][4], unreachable);
    EXPECT_EQ(holder_4.grid[2][0], unreachable);
    EXPECT_EQ(holder_4.grid[2][1], unreachable);
    EXPECT_EQ(holder_4.grid[2][2], unreachable);
    EXPECT_EQ(holder_4.grid[2][3], unreachable);
    EXPECT_EQ(holder_4.grid[2][4], unreachable);
    EXPECT_EQ(holder_4.grid[3][0], unreachable);
    EXPECT_EQ(holder_4.grid[3][1], unreachable);
    EXPECT_EQ(holder_4.grid[3][2], unreachable);
    EXPECT_EQ(holder_4.grid[3][3], unreachable);
    EXPECT_EQ(holder_4.grid[3][4], unreachable);
    EXPECT_EQ(holder_4.grid[4][0], unreachable);
    EXPECT_EQ(holder_4.grid[4][1], unreachable);
    EXPECT_EQ(holder_4.grid[4][2], unreachable);
    EXPECT_EQ(holder_4.grid[4][3], unreachable);
    EXPECT_EQ(holder_4.grid[4][4], unreachable);
}

TEST(DistanceTranformApprox, operability) {
    const float eps_3 = 0.41;  // DistanceTransformApprox3 uses Chamfer distance transform, while
                               // OpenCV uses Borgefors distance transform
    const float eps_5 = 0.2;

    const Size sz_1 = {.width = 10, .height = 10};
    const int res_1 = 1;
    const Pose pose_1({1, 2}, {3, 4});
    U8GridDataPtr data_1 = Allocate<uint8_t>(sz_1);

    std::fill(data_1.get(), data_1.get() + sz_1(), 1);

    U8Grid grid_1(sz_1, res_1, pose_1);
    grid_1.Reset(data_1.get());

    grid_1[1][4] = 0;
    grid_1[2][7] = 0;
    grid_1[3][7] = 0;
    grid_1[4][1] = 0;
    grid_1[5][5] = 0;
    grid_1[6][7] = 0;

    cv::Mat cv_grid_1(sz_1.height, sz_1.width, CV_8UC1);
    for (int i = 0; i < sz_1.height; ++i) {
        for (int j = 0; j < sz_1.width; ++j) {
            cv_grid_1.at<uchar>(i, j) = grid_1[i][j] == 0 ? 0 : 1;
        }
    }

    F32GridHolder result_3_1 = DistanceTransformApprox3(grid_1);
    auto cv_result_3_1 = cv::Mat();

    cv::distanceTransform(cv_grid_1, cv_result_3_1, cv::DIST_L2, cv::DIST_MASK_3);

    for (int i = 0; i < sz_1.height; ++i) {
        for (int j = 0; j < sz_1.width; ++j) {
            EXPECT_NEAR(result_3_1.grid[i][j], cv_result_3_1.at<float>(i, j), eps_3);
        }
    }

    F32GridHolder result_5_1 = DistanceTransformApprox5(grid_1);
    auto cv_result_5_1 = cv::Mat();

    cv::distanceTransform(cv_grid_1, cv_result_5_1, cv::DIST_L2, cv::DIST_MASK_5);

    for (int i = 0; i < sz_1.height; ++i) {
        for (int j = 0; j < sz_1.width; ++j) {
            EXPECT_NEAR(result_5_1.grid[i][j], cv_result_5_1.at<float>(i, j), eps_5);
        }
    }

    const Size sz_2 = {.width = 10, .height = 10};
    const int res_2 = 1;
    const Pose pose_2({1, 2}, {3, 4});
    U8GridDataPtr data_2 = Allocate<uint8_t>(sz_2);

    std::fill(data_2.get(), data_2.get() + sz_2(), 1);

    U8Grid grid_2(sz_2, res_2, pose_2);
    grid_2.Reset(data_2.get());

    grid_2[0][0] = 0;
    grid_2[9][9] = 0;

    cv::Mat cv_grid_2(sz_2.height, sz_2.width, CV_8UC1);
    for (int i = 0; i < sz_2.height; ++i) {
        for (int j = 0; j < sz_2.width; ++j) {
            cv_grid_2.at<uchar>(i, j) = grid_2[i][j] == 0 ? 0 : 1;
        }
    }

    F32GridHolder result_3_2 = DistanceTransformApprox3(grid_2);
    auto cv_result_3_2 = cv::Mat();

    cv::distanceTransform(cv_grid_2, cv_result_3_2, cv::DIST_L2, cv::DIST_MASK_3);

    for (int i = 0; i < sz_2.height; ++i) {
        for (int j = 0; j < sz_2.width; ++j) {
            EXPECT_NEAR(result_3_2.grid[i][j], cv_result_3_2.at<float>(i, j), eps_3);
        }
    }

    F32GridHolder result_5_2 = DistanceTransformApprox5(grid_2);
    auto cv_result_5_2 = cv::Mat();

    cv::distanceTransform(cv_grid_2, cv_result_5_2, cv::DIST_L2, cv::DIST_MASK_5);

    for (int i = 0; i < sz_2.height; ++i) {
        for (int j = 0; j < sz_2.width; ++j) {
            EXPECT_NEAR(result_5_2.grid[i][j], cv_result_5_2.at<float>(i, j), eps_5);
        }
    }
}

TEST(BilinearInterpolation, operability) {
    const double eps = 1e-8;

    const Size sz_1 = {.width = 2, .height = 2};
    const double res_1 = 1;
    F32GridDataPtr data_1 = Allocate<float>(sz_1);
    F32Grid grid_1(sz_1, res_1);
    grid_1.Reset(data_1.get());

    grid_1[0][0] = 0;
    grid_1[0][1] = 0.5;
    grid_1[1][0] = 0.5;
    grid_1[1][1] = 1;

    BilinearInterpolation<float> interpolation_1_1(grid_1);

    EXPECT_NEAR(interpolation_1_1({.x = 0.5, .y = 0.5}), 0.5, eps);

    grid_1[0][0] = 0;
    grid_1[0][1] = 0.5;
    grid_1[1][0] = 0.8;
    grid_1[1][1] = 1;

    BilinearInterpolation<float> interpolation_1_2(grid_1);

    EXPECT_NEAR(interpolation_1_2({.x = 0.5, .y = 0.5}), 0.575, eps);

    grid_1[0][0] = 0;
    grid_1[0][1] = 0.3;
    grid_1[1][0] = 0.8;
    grid_1[1][1] = 1;

    BilinearInterpolation<float> interpolation_1_3(grid_1);

    EXPECT_NEAR(interpolation_1_3({.x = 0.5, .y = 0.5}), 0.525, eps);

    grid_1[0][0] = 0;
    grid_1[0][1] = 0.4;
    grid_1[1][0] = 0.8;
    grid_1[1][1] = 1;

    BilinearInterpolation<float> interpolation_1_4(grid_1);

    EXPECT_NEAR(interpolation_1_4({.x = 0.2, .y = 0.6}), 0.376, eps);

    const Size sz_2 = {.width = 3, .height = 3};
    const double res_2 = 0.5;
    F32GridDataPtr data_2 = Allocate<float>(sz_2);
    F32Grid grid_2(sz_2, res_2);
    grid_2.Reset(data_2.get());

    grid_2[0][0] = 0;
    grid_2[0][1] = 0.5;
    grid_2[0][2] = 0.75;
    grid_2[1][0] = 0.5;
    grid_2[1][1] = 1;
    grid_2[1][2] = 1.5;
    grid_2[2][0] = 0;
    grid_2[2][1] = 1;
    grid_2[2][2] = 2;

    BilinearInterpolation<float> interpolation_2_1(grid_2);

    EXPECT_NEAR(interpolation_2_1({.x = 0.15, .y = 0.25}), 0.4, eps);
    EXPECT_NEAR(interpolation_2_1({.x = 0.65, .y = 0.25}), 0.8625, eps);
    EXPECT_NEAR(interpolation_2_1({.x = 0.65, .y = 0.65}), 1.195, eps);

    try {
        interpolation_2_1({.x = 1, .y = 1});
        FAIL();
    } catch (const std::domain_error& err) {
        EXPECT_EQ(err.what(), std::string("Point is not inside the grid"));
    } catch (...) {
        FAIL();
    }

    try {
        interpolation_2_1({.x = 100, .y = 0.5});
        FAIL();
    } catch (const std::domain_error& err) {
        EXPECT_EQ(err.what(), std::string("Point is not inside the grid"));
    } catch (...) {
        FAIL();
    }

    try {
        interpolation_2_1({.x = 0.54, .y = -1});
        FAIL();
    } catch (const std::domain_error& err) {
        EXPECT_EQ(err.what(), std::string("Point is not inside the grid"));
    } catch (...) {
        FAIL();
    }
}
