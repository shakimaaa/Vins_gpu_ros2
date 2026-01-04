/*******************************************************
 * Copyright (C) 2025, Xion Control
 *
 * This file is part of XVoyager.
 *
 * Licensed under the MIT License; you may not use this file
 * except in compliance with the License.
 *
 * See the LICENSE file distributed with this work for details.
 *
 * Developed by Xion Control.
 *******************************************************/

#include "xvins/featureTracker/feature_tracker.hpp"

namespace xvins
{

    bool FeatureTracker::inBorder(const cv::Point2f &pt)
    {
        const int BORDER_SIZE = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
    }

    // double distance(cv::Point2f pt1, cv::Point2f pt2)
    // {
    //     //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    //     double dx = pt1.x - pt2.x;
    //     double dy = pt1.y - pt2.y;
    //     return sqrt(dx * dx + dy * dy);
    // }

    double distance_2(const cv::Point2f &pt1, const cv::Point2f &pt2)
    {
        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        //double c = dx * dx + dy * dy;
        // ROS_INFO("dx %d", dx);
        // ROS_INFO("dy %d", dy);
        // ROS_INFO("c %d", c);
        return dx * dx + dy * dy;
    }

    void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    void reduceVector(vector<int> &v, vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    FeatureTracker::FeatureTracker()
    {
        stereo_cam = 0;
        n_id = 0;
        hasPrediction = false;
        sum_n = 0;
    }

    void FeatureTracker::setMask()
    {
        mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

        // prefer to keep features that are tracked for long time
        vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

        for (unsigned int i = 0; i < cur_pts.size(); i++)
            cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

        sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
             { return a.first > b.first; });

        cur_pts.clear();
        ids.clear();
        track_cnt.clear();

        for (auto &it : cnt_pts_id)
        {
            if (mask.at<uchar>(it.second.first) == 255)
            {
                cur_pts.push_back(it.second.first);
                ids.push_back(it.second.second);
                track_cnt.push_back(it.first);
                cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            }
        }
        // ROS_INFO("[FeatureTracker] mask ok");
    }

    void FeatureTracker::addPoints()
    {
        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
    }

    double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
    {
        // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        return sqrt(dx * dx + dy * dy);
    }

    // ORB_trackImage Deprecated
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
    {
        if (TRACK_METHOD == TrackingMethod::ORB)
        {
            return ORB_trackImage(_cur_time, _img, _img1);
        }
        else if (TRACK_METHOD == TrackingMethod::OPTICAL_FLOW)
        {
            return FLOW_trackImage(_cur_time, _img, _img1);
        }
        else
        {
            ROS_ERROR("Unknown tracking method");
            return {};
        }
    }

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::FLOW_trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
    {
        TicToc t_r;
        // Initialize the current frame time, image and right eye image
        // ROS_INFO("start trackImage");
        cur_time = _cur_time;
        cur_img = _img;
        row = cur_img.rows;
        col = cur_img.cols;
        cv::Mat rightImg = _img1;
        cur_pts.clear();

        cv::cuda::GpuMat cur_gpu_img;
        cv::cuda::GpuMat right_gpu_Img;
        // If GPU accelerated optical flow is enabled, initializes the GPU image and optical flow objects
        if (USE_GPU_ACC_FLOW)
        {
            // ROS_INFO("[FeatureTracker] use gpu");
            cur_gpu_img = cv::cuda::GpuMat(cur_img);
            right_gpu_Img = cv::cuda::GpuMat(rightImg);
            if (d_pyrLK_sparse.empty())
                d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), LK_N, 30, false);
            if (d_pyrLK_sparse_prediction.empty())
                d_pyrLK_sparse_prediction = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), 1, 30, true);
        }

        // If there are feature points in the previous frame, perform optical flow tracking
        if (prev_pts.size() > 0)
        {
            // ROS_INFO("[FeatureTracker] prev_pts.size > 0");
            vector<uchar> status;
            // CPU optical flow tracking
            if (!USE_GPU_ACC_FLOW)
            {   
                TicToc T_Flow;
                vector<float> err;
                if (hasPrediction)
                {
                    // If there is a prediction point, use the prediction point as the initial value
                    cur_pts = predict_pts;
                    cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(LK_SIZE, LK_SIZE), 1, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                    int succ_num = 0;
                    for (size_t i = 0; i < status.size(); i++)
                    {
                        if (status[i])
                            succ_num++;
                    }
                    // If the number of successful tracking points is too few, perform optical flow tracking again.
                    if (succ_num < 10)
                        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(LK_SIZE, LK_SIZE), LK_N);
                }
                else
                    cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(LK_SIZE, LK_SIZE), LK_N);
                ROS_INFO("[feature_tracker] CPU Forward optical flow cost: %f\nms", T_Flow.toc());
                    // Reverse Check to eliminate false matches
                if (FLOW_BACK)
                {
                    TicToc t_Flow_Back;
                    vector<uchar> reverse_status;
                    vector<cv::Point2f> reverse_pts = prev_pts;
                    cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(LK_SIZE, LK_SIZE), 1, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                    // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(LK_SIZE, LK_SIZE), 3);
                    for (size_t i = 0; i < status.size(); i++)
                    {
                        if (status[i] && reverse_status[i] && distance_2(prev_pts[i], reverse_pts[i]) <= 0.25)
                            status[i] = 1;
                        else
                            status[i] = 0;
                    }
                    ROS_INFO("[feature_tracker] CUP optical flow back cost: %fms\n", t_Flow_Back.toc());
                }
                // printf("temporal optical flow costs: %fms\n", t_o.toc());
            }
            // GPU optical flow tracking
            else
            {
                TicToc t_og;
                //  cv::cuda::GpuMat cur_gpu_img(cur_img);
                //  cv::cuda::GpuMat prev_gpu_img(prev_img);
                cv::cuda::GpuMat prev_gpu_pts(prev_pts);
                cv::cuda::GpuMat cur_gpu_pts(cur_pts);
                cv::cuda::GpuMat gpu_status;
                if (hasPrediction)
                {
                    cur_gpu_pts = cv::cuda::GpuMat(predict_pts);
                    // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse_prediction = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), 1, 30, true);
                    d_pyrLK_sparse_prediction->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

                    vector<cv::Point2f> tmp_cur_pts(cur_gpu_pts.cols);
                    cur_gpu_pts.download(tmp_cur_pts);
                    cur_pts = tmp_cur_pts;

                    vector<uchar> tmp_status(gpu_status.cols);
                    gpu_status.download(tmp_status);
                    status = tmp_status;

                    int succ_num = 0;
                    for (size_t i = 0; i < tmp_status.size(); i++)
                    {
                        if (tmp_status[i])
                            succ_num++;
                    }
                    if (succ_num < 10)
                    {
                        // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), LK_N, 30, false);
                        d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

                        vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
                        cur_gpu_pts.download(tmp1_cur_pts);
                        cur_pts = tmp1_cur_pts;

                        vector<uchar> tmp1_status(gpu_status.cols);
                        gpu_status.download(tmp1_status);
                        status = tmp1_status;
                    }
                }
                else
                {
                    // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), LK_N, 30, false);
                    d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

                    vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
                    cur_gpu_pts.download(tmp1_cur_pts);
                    cur_pts = tmp1_cur_pts;

                    vector<uchar> tmp1_status(gpu_status.cols);
                    gpu_status.download(tmp1_status);
                    status = tmp1_status;
                }
                // GPU reverse check
                if (FLOW_BACK)
                {
                    cv::cuda::GpuMat reverse_gpu_status;
                    cv::cuda::GpuMat reverse_gpu_pts = prev_gpu_pts;
                    // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse_prediction = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), 1, 30, true);
                    d_pyrLK_sparse_prediction->calc(cur_gpu_img, prev_gpu_img, cur_gpu_pts, reverse_gpu_pts, reverse_gpu_status);

                    vector<cv::Point2f> reverse_pts(reverse_gpu_pts.cols);
                    reverse_gpu_pts.download(reverse_pts);

                    vector<uchar> reverse_status(reverse_gpu_status.cols);
                    reverse_gpu_status.download(reverse_status);

                    for (size_t i = 0; i < status.size(); i++)
                    {
                        if (status[i] && reverse_status[i] && distance_2(prev_pts[i], reverse_pts[i]) <= 0.25)
                        {
                            status[i] = 1;
                        }
                        else
                            status[i] = 0;
                    }
                }
                ROS_INFO("[feature_tracker] gpu temporal optical flow costs: %f ms\n",t_og.toc());
                // ROS_INFO("[FeatureTracker] gpu end");
            }

            // Cull points that fail to track or exceed boundaries
            for (int i = 0; i < int(cur_pts.size()); i++)
                if (status[i] && !inBorder(cur_pts[i]))
                    status[i] = 0;
            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
        }

        // Update trace count
        for (auto &n : track_cnt)
            n++;

        if (1)
        {
            // Set a mask to avoid extracting new feature points near existing feature points
            setMask();
            //ROS_INFO("[FeatureTracker] cur_pts %d", cur_pts.size());
            int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
            //ROS_INFO("[FeatureTracker] n_max_cnt= %d", n_max_cnt);
            //If the number of feature points is insufficient, extract new feature points
            if (!USE_GPU)
            {
                if (n_max_cnt > 0)
                {
                    TicToc t_t;
                    if (mask.empty())
                        cout << "mask is empty " << endl;
                    if (mask.type() != CV_8UC1)
                        cout << "mask type wrong " << endl;
                    // CPU extracts feature points
                    cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
                    // printf("good feature to track costs: %fms\n", t_t.toc());
                    // std::cout << "n_pts size: " << n_pts.size() << std::endl;
                    ROS_INFO("good feature to track costs: %fms\n", t_t.toc());
                }
                else
                    n_pts.clear();
                sum_n += n_pts.size();
                ROS_INFO("total point from non-gpu: %d\n",sum_n);
            }

            // ROS_DEBUG("detect feature costs: %fms", t_t.toc());
            
            else
            {
                if (n_max_cnt > 0)
                {
                    // ROS_INFO("n_max_cnt > 0");
                    if (mask.empty())
                        // cout << "mask is empty " << endl;
                        ROS_INFO("mask is empty");
                    if (mask.type() != CV_8UC1)
                        // cout << "mask type wrong " << endl;
                        ROS_INFO("mask type wrong");
                    TicToc t_g;
                    cv::cuda::GpuMat cur_gpu_img(cur_img);
                    cv::cuda::GpuMat d_prevPts;
                    TicToc t_gg;
                    cv::cuda::GpuMat gpu_mask(mask);
                    // ROS_INFO("[FeatureTracker] gpu_mask");
                    ROS_INFO("[feature_tracker] gpumat cost: %fms\n",t_gg.toc());
                    // GPU Extract feature points
                    cv::Ptr<cv::cuda::CornersDetector> detector = cv::cuda::createGoodFeaturesToTrackDetector(cur_gpu_img.type(), MAX_CNT - cur_pts.size(), 0.01, MIN_DIST);
                    // cout << "new gpu points: "<< MAX_CNT - cur_pts.size()<<endl;
                    detector->detect(cur_gpu_img, d_prevPts, gpu_mask);
                    // std::cout << "d_prevPts size: "<< d_prevPts.size()<<std::endl;
                    if (!d_prevPts.empty())
                    {
                        n_pts = cv::Mat_<cv::Point2f>(cv::Mat(d_prevPts));
                        // ROS_INFO("[FeatureTracker] get n_pts");
                    }
                    else
                        n_pts.clear();
                    sum_n += n_pts.size();
                    ROS_INFO("[feature_tracker] total point from gpu: %d\n",sum_n);
                    ROS_INFO("[feature_tracker] gpu good feature to track cost: %fms\n", t_g.toc());
                }
                else
                    n_pts.clear();
            }

            // ROS_INFO("[FeatureTracker] add feature begins");
            TicToc t_a;
            // Add the newly extracted feature points to the tracking list
            addPoints();
            // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
            ROS_INFO("[feature_tracker] selectFeature costs: %fms\n", t_a.toc());
        }

        // Remove distortion and calculate feature point velocity
        cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
        pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

        // If it is a binocular camera, process the right eye image
        if (!_img1.empty() && stereo_cam)
        {
            ids_right.clear();
            cur_right_pts.clear();
            cur_un_right_pts.clear();
            right_pts_velocity.clear();
            cur_un_right_pts_map.clear();
            if (!cur_pts.empty())
            {
                // printf("stereo image; track feature on right image\n");
                ROS_INFO("[FeatureTracker] stereo image; track feature on right image\n");

                vector<cv::Point2f> reverseLeftPts;
                vector<uchar> status, statusRightLeft;
                // CPU binocular flow tracking (left picture -> right picture)
                if (!USE_GPU_ACC_FLOW)
                {
                    // TicToc t_check;
                    vector<float> err;
                    // cur left ---- cur right
                    cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(LK_SIZE, LK_SIZE), LK_N);
                    // reverse check cur right ---- cur left
                    if (FLOW_BACK)
                    {
                        // cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(LK_SIZE, LK_SIZE), LK_N);
                        reverseLeftPts = cur_pts;
                        cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(LK_SIZE, LK_SIZE), 1, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                        for (size_t i = 0; i < status.size(); i++)
                        {
                            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance_2(cur_pts[i], reverseLeftPts[i]) <= 0.25)
                                status[i] = 1;
                            else
                                status[i] = 0;
                        }
                    }
                    // printf("left right optical flow cost %fms\n",t_check.toc());
                }
                // GPU binocular flow tracking (left picture -> right picture)
                else
                {
                    TicToc t_og1;
                    // cv::cuda::GpuMat cur_gpu_img(cur_img);
                    // cv::cuda::GpuMat right_gpu_Img(rightImg);
                    cv::cuda::GpuMat cur_gpu_pts(cur_pts);
                    cv::cuda::GpuMat cur_right_gpu_pts;
                    cv::cuda::GpuMat gpu_status;
                    // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), LK_N, 30, false);
                    d_pyrLK_sparse->calc(cur_gpu_img, right_gpu_Img, cur_gpu_pts, cur_right_gpu_pts, gpu_status);

                    vector<cv::Point2f> tmp_cur_right_pts(cur_right_gpu_pts.cols);
                    cur_right_gpu_pts.download(tmp_cur_right_pts);
                    cur_right_pts = tmp_cur_right_pts;
                    // ROS_INFO("[FeatureTracker] cur_right_pts size: %d", cur_right_pts.size());

                    vector<uchar> tmp_status(gpu_status.cols);
                    gpu_status.download(tmp_status);
                    status = tmp_status;

                    if (FLOW_BACK)
                    {
                        cv::cuda::GpuMat reverseLeft_gpu_Pts;
                        cv::cuda::GpuMat status_gpu_RightLeft;
                        // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), LK_N, 30, false);
                        // reverseLeft_gpu_Pts = cur_gpu_pts;
                        // reverseLeft_gpu_Pts.copyTo(cur_gpu_pts);
                        cur_right_gpu_pts.copyTo(reverseLeft_gpu_Pts);
                        // cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse_prediction = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(LK_SIZE, LK_SIZE), 1, 30, true);
                        d_pyrLK_sparse_prediction->calc(right_gpu_Img, cur_gpu_img, cur_right_gpu_pts, reverseLeft_gpu_Pts, status_gpu_RightLeft);

                        vector<cv::Point2f> tmp_reverseLeft_Pts(reverseLeft_gpu_Pts.cols);
                        reverseLeft_gpu_Pts.download(tmp_reverseLeft_Pts);
                        reverseLeftPts = tmp_reverseLeft_Pts;
                        // cur_right_gpu_pts.copyTo(cur_gpu_pts);

                        vector<uchar> tmp1_status(status_gpu_RightLeft.cols);
                        status_gpu_RightLeft.download(tmp1_status);
                        statusRightLeft = tmp1_status;
                        // for(size_t i = 0; i < status.size(); i++)
                        // {
                        //     ROS_INFO("status: %d",status[i]);
                        //     break;
                        // }

                        for (size_t i = 0; i < status.size(); i++)
                        {
                            // ROS_INFO("427 cur_right_pts: %d",cur_right_pts[i]);
                            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance_2(cur_pts[i], reverseLeftPts[i]) <= 0.25)

                                status[i] = 1;
                            else
                                status[i] = 0;
                        }
                    }
                    ROS_INFO("[feature_tracker] gpu left right optical flow cost %fms\n",t_og1.toc());
                }
                ids_right = ids;
                reduceVector(cur_right_pts, status);
                reduceVector(ids_right, status);
                // only keep left-right pts
                /*
                reduceVector(cur_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(cur_un_pts, status);
                reduceVector(pts_velocity, status);
                */
                // De-distort the feature points of the right eye and calculate the speed
                cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
                right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
            }
            prev_un_right_pts_map = cur_un_right_pts_map;
        }
        if (SHOW_TRACK)
        {
            drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        }

        // Update the previous frame data
        prev_img = cur_img;
        prev_pts = cur_pts;

        //ROS_INFO("[FeatureTracker] cur_pts size: %d", cur_pts.size());

        prev_un_pts = cur_un_pts;
        prev_un_pts_map = cur_un_pts_map;
        prev_time = cur_time;
        hasPrediction = false;

        if (USE_GPU_ACC_FLOW)
            prev_gpu_img = cur_gpu_img;

        prevLeftPtsMap.clear();
        for (size_t i = 0; i < cur_pts.size(); i++)
            prevLeftPtsMap[ids[i]] = cur_pts[i];

        // Encapsulate feature point data into featureFrame
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
        for (size_t i = 0; i < ids.size(); i++)
        {
            if (std::isnan(cur_un_pts[i].x) || std::isnan(cur_un_pts[i].y))
                continue;
            int feature_id = ids[i];
            double x, y, z;
            x = cur_un_pts[i].x;
            y = cur_un_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_pts[i].x;
            p_v = cur_pts[i].y;
            int camera_id = 0;
            double velocity_x, velocity_y;
            velocity_x = pts_velocity[i].x;
            velocity_y = pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }

        if (!_img1.empty() && stereo_cam)
        {
            for (size_t i = 0; i < ids_right.size(); i++)
            {
                if (std::isnan(cur_un_right_pts[i].x) || std::isnan(cur_un_right_pts[i].y))
                    continue;
                int feature_id = ids_right[i];
                double x, y, z;
                x = cur_un_right_pts[i].x;
                y = cur_un_right_pts[i].y;
                z = 1;
                double p_u, p_v;
                p_u = cur_right_pts[i].x;
                p_v = cur_right_pts[i].y;
                int camera_id = 1;
                double velocity_x, velocity_y;
                velocity_x = right_pts_velocity[i].x;
                velocity_y = right_pts_velocity[i].y;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
        }

        ROS_INFO("[FeatureTracker] feature track whole time %fms\n", t_r.toc());
        // printf("feature track whole time %f\n", t_r.toc());
        return featureFrame;
    }

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::ORB_trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
    {
        TicToc t_r;
        ROS_INFO("use  ORB_trackImage");
        cur_time = _cur_time;
        cur_img = _img;
        row = cur_img.rows;
        col = cur_img.cols;
        cv::Mat rightImg = _img1;
        cur_pts.clear();

        cv::cuda::GpuMat cur_gpu_img, right_gpu_Img;
        cv::cuda::GpuMat d_prev_kps, d_prev_desc, d_cur_kps, d_cur_desc;

        static cv::Ptr<cv::ORB> orb = cv::ORB::create(NFEATURES, SCALE_FACTOR, NLEVELS, EDGE_THRESHOLD, FIRST_LEVEL, WTA_K, cv::ORB::HARRIS_SCORE, PATCH_SIZE, FAST_THRESHOLD);

        if (USE_GPU)
        {
            cur_gpu_img = cv::cuda::GpuMat(cur_img);
            right_gpu_Img = cv::cuda::GpuMat(rightImg);
            if (orb_detector_gpu.empty())
                orb_detector_gpu = cv::cuda::ORB::create(NFEATURES, SCALE_FACTOR, NLEVELS, EDGE_THRESHOLD, FIRST_LEVEL, WTA_K, cv::ORB::HARRIS_SCORE, PATCH_SIZE, FAST_THRESHOLD);
            if (orb_matcher_gpu.empty())
                orb_matcher_gpu = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
        }
        if (prev_pts.size() > 0)
        {
            vector<uchar> status;
            if (!USE_GPU_ACC_FLOW)
            {
                // Convert the previous frame point to KeyPoints format
                vector<cv::KeyPoint> prev_kps;
                cv::KeyPoint::convert(prev_pts, prev_kps);

                // Calculate the previous frame descriptor
                cv::Mat prev_desc;
                orb->compute(prev_img, prev_kps, prev_desc);

                // Detect the current frame features
                vector<cv::KeyPoint> cur_kps;
                cv::Mat cur_desc;
                orb->detectAndCompute(cur_img, cv::noArray(), cur_kps, cur_desc);

                // Feature Matching
                vector<cv::DMatch> matches;
                cv::BFMatcher matcher(cv::NORM_HAMMING);
                matcher.match(prev_desc, cur_desc, matches);

                // Calculate the minimum distance
                double min_dist = min_element(matches.begin(), matches.end(),
                                              [](const cv::DMatch &a, const cv::DMatch &b)
                                              { return a.distance < b.distance; })
                                      ->distance;

                // Less than twice the calculated minimum distance or 10 is considered a good match.
                vector<cv::DMatch> good_matches;
                for (const auto &m : matches)
                {
                    if (m.distance < max(2* min_dist, ORB_MIN_DIST))
                    {
                        good_matches.push_back(m);
                    }
                }

                // Update the feature points of the current frame
                cur_pts.clear();
                for (const auto &m : good_matches)
                {
                    cur_pts.push_back(cur_kps[m.trainIdx].pt);
                }

                // Update tracking status
                status = vector<uchar>(prev_pts.size(), 0);
                for (const auto &m : good_matches)
                {
                    if (static_cast<size_t>(m.queryIdx) < prev_pts.size() &&
                        static_cast<size_t>(m.trainIdx) < cur_kps.size())
                    {
                        cur_pts.push_back(cur_kps[m.trainIdx].pt);
                        status[m.queryIdx] = 1;
                    }
                }

                // // Remove unqualified points
                // reduceVector(prev_pts, status);
                // reduceVector(ids, status);
                // reduceVector(track_cnt, status);
            }
            else if(!prev_pts.empty())
            {
                // GPU
                // Upload the previous frame point
                cv::Mat prev_pts_mat(prev_pts);
                d_prev_kps.upload(prev_pts_mat);

                // Calculate the previous frame descriptor
                orb_detector_gpu->computeAsync(prev_gpu_img, d_prev_kps, d_prev_desc);

                // Detect the current frame
                orb_detector_gpu->detectAndComputeAsync(cur_gpu_img, cv::noArray(), d_cur_kps, d_cur_desc);

                // Download key points
                vector<cv::KeyPoint> cur_kps;
                orb_detector_gpu->convert(d_cur_kps, cur_kps);

                // Feature Matching
                std::vector<cv::DMatch> matches;
                orb_matcher_gpu->match(d_prev_desc, d_cur_desc, matches);

                // Calculate the minimum distance
                double min_dist = min_element(matches.begin(), matches.end(),
                                              [](const cv::DMatch &a, const cv::DMatch &b)
                                              { return a.distance < b.distance; })
                                      ->distance;

                vector<cv::DMatch> good_matches;
                for (const auto &m : matches)
                {
                    if (m.distance < max(2* min_dist, ORB_MIN_DIST))
                    {
                        good_matches.push_back(m);
                    }
                }

                // Update current frame
                cur_pts.clear();
                status = vector<uchar>(prev_pts.size(), 0);
                for (const auto &m : good_matches)
                {
                    if (static_cast<size_t>(m.queryIdx) < prev_pts.size() &&
                        static_cast<size_t>(m.trainIdx) < cur_kps.size())
                    {
                        cur_pts.push_back(cur_kps[m.trainIdx].pt);
                        status[m.queryIdx] = 1;
                    }
                }
            }

            // Remove points where tracking failed
            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
        }

        for (auto &n : track_cnt)
            n++;

        if (1)
        {
            setMask();
            RCLCPP_INFO(rclcpp::get_logger("FeatureTracker")," cur_pts %ld", cur_pts.size());
            int n_max_cnt = NFEATURES - static_cast<int>(cur_pts.size());

            if (!USE_GPU)
            {
                if (n_max_cnt > 0)
                {
                    TicToc t_t;
                    if (mask.empty())
                        cout << "mask is empty " << endl;
                    if (mask.type() != CV_8UC1)
                        cout << "mask type wrong " << endl;

                    std::vector<cv::KeyPoint> new_kps;
                    orb->detect(cur_img, new_kps, mask);
                    n_pts.reserve(new_kps.size()); 
                    for (const auto& kp : new_kps) {
                        n_pts.push_back(kp.pt); 
                    }
                }
                else
                {
                    n_pts.clear();
                }
            }
            else
            {
                if(n_max_cnt > 0)
                {
                    ROS_INFO("[FeatureTracker] n_max_cnt %d", n_max_cnt);
                    if(mask.empty())
                        // cout << "mask is empty " << endl;
                        ROS_INFO("mask is empty");
                    if (mask.type() != CV_8UC1)
                        // cout << "mask type wrong " << endl;
                        ROS_INFO("mask type wrong");
                        TicToc t_g;
                        cv::cuda::GpuMat gpu_mask(mask);
                        // ROS_INFO("[ORB FeatureTracker] start detect feature on gpu");
                        cv::cuda::Stream stream;
                        orb_detector_gpu->detectAsync(cur_gpu_img, d_prev_kps, gpu_mask, stream);
                        stream.waitForCompletion();
                        cv::Mat cpu_prev_kps_mat;
                        d_prev_kps.download(cpu_prev_kps_mat);
                        // ROS_INFO("[ORB FeatureTracker] done detect feature on gpu");
                        if (!cpu_prev_kps_mat.empty() && cpu_prev_kps_mat.type() == CV_32FC2) {
                            // 将Mat转换为vector<Point2f>
                            n_pts.assign((cv::Point2f*)cpu_prev_kps_mat.datastart, 
                                         (cv::Point2f*)cpu_prev_kps_mat.dataend);
                        } else if (!cpu_prev_kps_mat.empty() && cpu_prev_kps_mat.type() == CV_32FC1) {
                            // 如果关键点是单通道浮点格式，需要reshape
                            cv::Mat reshaped = cpu_prev_kps_mat.reshape(2, cpu_prev_kps_mat.rows);
                            n_pts.assign((cv::Point2f*)reshaped.datastart, 
                                         (cv::Point2f*)reshaped.dataend);
                        }
                        else
                            n_pts.clear();
                }
                else
                {
                n_pts.clear();
                }
            }
            
            ROS_INFO("[FeatureTracker] add feature begins");
            addPoints();
        }

        cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
        pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

        if(!_img1.empty() && stereo_cam)
        {
            ids_right.clear();
            cur_right_pts.clear();
            cur_un_right_pts.clear();
            right_pts_velocity.clear();
            cur_un_right_pts_map.clear();
            if(!cur_pts.empty())
            {
                ROS_INFO("[FeatureTracker] stereo image; track feature on right image\n");

                vector<cv::Point2f> reverseLeftPts;
                vector<uchar> status, statusRightLeft;
                if(!USE_GPU_ACC_FLOW)
                {
                    // 将左图当前特征点转为KeyPoint格式
                    vector<cv::KeyPoint> left_kps;
                    cv::KeyPoint::convert(cur_pts, left_kps);

                    // 计算左图当前特征点描述子
                    cv::Mat left_desc;
                    orb->compute(cur_img, left_kps, left_desc);

                    // 检测右图特征点
                    vector<cv::KeyPoint> right_kps;
                    cv::Mat right_desc;
                    orb->detectAndCompute(rightImg, cv::noArray(), right_kps, right_desc);
                    // 特征匹配
                    vector<cv::DMatch> matches;
                    cv::BFMatcher matcher(cv::NORM_HAMMING);
                    matcher.match(left_desc, right_desc, matches);

                    // 筛选优质匹配
                    double min_dist = min_element(matches.begin(), matches.end(), 
                    [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; })->distance;

                    vector<cv::DMatch> good_matches;
                    for (const auto& m : matches) {
                        if (m.distance < max(2*min_dist, ORB_MIN_DIST)) {
                            good_matches.push_back(m);
                        }
                    }
                    // 更新右图特征点
                    cur_right_pts.clear();
                    status = vector<uchar>(cur_pts.size(), 0);
                    for (const auto& m : good_matches) {
                        if (static_cast<size_t>(m.queryIdx) < cur_pts.size() && static_cast<size_t>(m.trainIdx) < right_kps.size()) {
                            cur_right_pts.push_back(right_kps[m.trainIdx].pt);
                            status[m.queryIdx] = 1;
                        }
                    }
                }
                else
                {
                    // GPU
                    // 上传左图当前特征点
                    cv::cuda::GpuMat d_left_desc, d_right_desc;
                    cv::cuda::GpuMat d_left_kps, d_right_kps;
                    cv::Mat cur_pts_mat(cur_pts);
                    d_left_kps.upload(cur_pts_mat);
                    // 计算左图描述子
                    orb_detector_gpu->computeAsync(cur_gpu_img, d_left_kps, d_left_desc);
                    // 检测右图特征
                    orb_detector_gpu->detectAndComputeAsync(right_gpu_Img, cv::noArray(), d_right_kps, d_right_desc);
                    // 下载右图关键点
                    vector<cv::KeyPoint> right_kps;
                    orb_detector_gpu->convert(d_right_kps, right_kps);
                    
                    // 特征匹配
                    std::vector<cv::DMatch> matches;
                    orb_matcher_gpu->match(d_left_desc, d_left_desc, matches);

                    // 筛选优质匹配
                    double min_dist = min_element(matches.begin(), matches.end(), 
                    [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; })->distance;

                    vector<cv::DMatch> good_matches;
                    for (const auto& m : matches) {
                        if (m.distance < max(2*min_dist, ORB_MIN_DIST)) {
                            good_matches.push_back(m);
                        }
                    }

                    // 更新右图特征点
                    cur_right_pts.clear();
                    status = vector<uchar>(cur_pts.size(), 0);
                    for (const auto& m : good_matches) {
                        if (static_cast<size_t>(m.queryIdx) < cur_pts.size() && static_cast<size_t>(m.trainIdx) < right_kps.size()) {
                            cur_right_pts.push_back(right_kps[m.trainIdx].pt);
                            status[m.queryIdx] = 1;
                        }
                    }
                }
                ids_right = ids;
                reduceVector(cur_right_pts, status);
                reduceVector(ids_right, status);

                cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
                right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
            }
            prev_un_right_pts_map = cur_un_right_pts_map;
        }
        if (SHOW_TRACK)
        {
            drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        }

        prev_img = cur_img;
        prev_pts = cur_pts;
        if (USE_GPU_ACC_FLOW)
            prev_gpu_img = cur_gpu_img;
        
        prev_un_pts = cur_un_pts;
        prev_un_pts_map = cur_un_pts_map;
        prev_time = cur_time;
        // hasPrediction = false;

        prevLeftPtsMap.clear();
        for(size_t i = 0; i < cur_pts.size(); i++) {
            prevLeftPtsMap[ids[i]] = cur_pts[i];
        }

        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
        for (size_t i = 0; i < ids.size(); i++) {
            if(std::isnan(cur_un_pts[i].x) || std::isnan(cur_un_pts[i].y))
                continue;
            
            int feature_id = ids[i];
            double x = cur_un_pts[i].x;
            double y = cur_un_pts[i].y;
            double z = 1;
            double p_u = cur_pts[i].x;
            double p_v = cur_pts[i].y;
            int camera_id = 0;
            double velocity_x = pts_velocity[i].x;
            double velocity_y = pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }

        if (!_img1.empty() && stereo_cam) {
            for (size_t i = 0; i < ids_right.size(); i++) {
                if(std::isnan(cur_un_right_pts[i].x) || std::isnan(cur_un_right_pts[i].y))
                    continue;
                
                int feature_id = ids_right[i];
                double x = cur_un_right_pts[i].x;
                double y = cur_un_right_pts[i].y;
                double z = 1;
                double p_u = cur_right_pts[i].x;
                double p_v = cur_right_pts[i].y;
                int camera_id = 1;
                double velocity_x = right_pts_velocity[i].x;
                double velocity_y = right_pts_velocity[i].y;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
        }

        ROS_INFO("[FeatureTracker] feature track whole time %f\nms", t_r.toc());
        return featureFrame;
    }

    void FeatureTracker::rejectWithF()
    {
        if (cur_pts.size() >= 8)
        {
            ROS_DEBUG("FM ransac begins");
            TicToc t_f;
            vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
            for (unsigned int i = 0; i < cur_pts.size(); i++)
            {
                Eigen::Vector3d tmp_p;
                m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
                un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
                un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }

            vector<uchar> status;
            cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
            int size_a = cur_pts.size();
            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(cur_un_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
            ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
        }
    }

    void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
    {
        for (size_t i = 0; i < calib_file.size(); i++)
        {
            ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
            camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
            m_camera.push_back(camera);
        }
        if (calib_file.size() == 2)
            stereo_cam = 1;
    }

    void FeatureTracker::showUndistortion(const string &name)
    {
        cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
        vector<Eigen::Vector2d> distortedp, undistortedp;
        for (int i = 0; i < col; i++)
            for (int j = 0; j < row; j++)
            {
                Eigen::Vector2d a(i, j);
                Eigen::Vector3d b;
                m_camera[0]->liftProjective(a, b);
                distortedp.push_back(a);
                undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
                // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
            }
        for (int i = 0; i < int(undistortedp.size()); i++)
        {
            cv::Mat pp(3, 1, CV_32FC1);
            pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
            pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
            pp.at<float>(2, 0) = 1.0;
            // cout << trackerData[0].K << endl;
            // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
            // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
            if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
            {
                undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
            }
            else
            {
                // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
            }
        }
        cv::imshow(name, undistortedImg);
        cv::waitKey(0);
    }

    vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
    {
        vector<cv::Point2f> un_pts;
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            Eigen::Vector2d a(pts[i].x, pts[i].y);
            Eigen::Vector3d b;
            cam->liftProjective(a, b);
            un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        }
        RCLCPP_INFO(rclcpp::get_logger("FeatureTracker"),"un_pts size: %ld", un_pts.size());
        return un_pts;
    }

    vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
    {
        vector<cv::Point2f> pts_velocity;
        cur_id_pts.clear();
        for (unsigned int i = 0; i < ids.size(); i++)
        {
            cur_id_pts.insert(make_pair(ids[i], pts[i]));
        }

        // caculate points velocity
        if (!prev_id_pts.empty())
        {
            double dt = cur_time - prev_time;

            for (unsigned int i = 0; i < pts.size(); i++)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_id_pts.find(ids[i]);
                if (it != prev_id_pts.end())
                {
                    double v_x = (pts[i].x - it->second.x) / dt;
                    double v_y = (pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
        else
        {
            for (unsigned int i = 0; i < cur_pts.size(); i++)
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
        return pts_velocity;
    }

    void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts,
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap)
    {
        // int rows = imLeft.rows;
        int cols = imLeft.cols;
        if (!imRight.empty() && stereo_cam)
            cv::hconcat(imLeft, imRight, imTrack);
        else
            imTrack = imLeft.clone();
        cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

        for (size_t j = 0; j < curLeftPts.size(); j++)
        {
            double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
            cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        if (!imRight.empty() && stereo_cam)
        {
            RCLCPP_INFO(rclcpp::get_logger("FeatureTracker"),"curRightPts size %ld", curRightPts.size());
            for (size_t i = 0; i < curRightPts.size(); i++)
            {
                cv::Point2f rightPt = curRightPts[i];
                rightPt.x += cols;
                cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
                // cv::Point2f leftPt = curLeftPtsTrackRight[i];
                // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
            }
        }

        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curLeftIds.size(); i++)
        {
            int id = curLeftIds[i];
            mapIt = prevLeftPtsMap.find(id);
            if (mapIt != prevLeftPtsMap.end())
            {
                cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
            }
        }

        // draw prediction
        /*
        for(size_t i = 0; i < predict_pts_debug.size(); i++)
        {
            cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
        }
        */
        // printf("predict pts size %d \n", (int)predict_pts_debug.size());

        // cv::Mat imCur2Compress;
        // cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));

        // cv::imshow("tracking", imTrack);
        // cv::waitKey(2);
    }

    void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
    {
        hasPrediction = true;
        predict_pts.clear();
        predict_pts_debug.clear();
        map<int, Eigen::Vector3d>::iterator itPredict;
        for (size_t i = 0; i < ids.size(); i++)
        {
            // printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
            int id = ids[i];
            itPredict = predictPts.find(id);
            if (itPredict != predictPts.end())
            {
                Eigen::Vector2d tmp_uv;
                m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
                predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
                predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            }
            else
                predict_pts.push_back(prev_pts[i]);
        }
    }

    void FeatureTracker::removeOutliers(set<int> &removePtsIds)
    {
        std::set<int>::iterator itSet;
        vector<uchar> status;
        for (size_t i = 0; i < ids.size(); i++)
        {
            itSet = removePtsIds.find(ids[i]);
            if (itSet != removePtsIds.end())
                status.push_back(0);
            else
                status.push_back(1);
        }

        reduceVector(prev_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }

    cv::Mat FeatureTracker::getTrackImage()
    {
        return imTrack;
    }

} // namespace xvins
