#include <sys/types.h>
#include <Eigen/Dense>
#include "YAstar/yastar.hpp"
#include "YAstar/kinodynamicAstar.hpp"
#include "MinimumSnapOsqp/minimumSnap.hpp"
#include "MinimumSnapOsqp/sfcSquare.hpp"
#include <chrono>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>
#include <random>

static bool g_show_gui = false;
static std::string g_root_dir = "/home/wgk/decision_planning/SCU-TDT-V1";
int pass = 0;
int fail = 0;
static std::map<std::string, std::tuple<int, double, double>> record;

template<typename Func>
auto benchTime(const std::string& name, Func&& func) -> decltype(func()) {
    auto start = std::chrono::high_resolution_clock::now();
    auto result = std::forward<Func>(func)();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(end - start).count();
    std::cout << name << " time: " << duration * 1e6 << " us" << std::endl;
    if(record.find(name) == record.end()){
        record[name] = std::make_tuple(1, duration, duration);
    }else{
        int count = std::get<0>(record[name]);
        double avg = std::get<1>(record[name]);
        double max = std::get<2>(record[name]);
        count++;
        avg += (duration - avg) / static_cast<double>(count);
        max = std::max(max, static_cast<double>(duration));
        record[name] = std::make_tuple(count, avg, max);
    }
    return result;
}

template<typename Func>
void benchTimeVoid(const std::string& name, Func&& func) {
    auto start = std::chrono::high_resolution_clock::now();
    std::forward<Func>(func)();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(end - start).count();
    std::cout << name << " time: " << duration * 1e6 << " us" << std::endl;
    if(record.find(name) == record.end()){
        record[name] = std::make_tuple(1, duration, duration);
    }else{
        int count = std::get<0>(record[name]);
        double avg = std::get<1>(record[name]);
        double max = std::get<2>(record[name]);
        count++;
        avg += (duration - avg) / static_cast<double>(count);
        max = std::max(max, static_cast<double>(duration));
        record[name] = std::make_tuple(count, avg, max);
    }
}

double test(Eigen::Vector2f start, Eigen::Vector2f end,
            const std::string& map_file = "rmuc2025.png",
            const std::string& save_out = "") {
    std::cout << "from [" << start.x() << ", " << start.y() << "] to [" << end.x() << ", " << end.y() << "] on " << map_file << std::endl;
    cv::Mat map = cv::imread(g_root_dir + "/images/" + map_file, cv::IMREAD_GRAYSCALE);
    if(map.empty()){
        std::cerr << "无法读取地图: " << g_root_dir + "/images/" + map_file << std::endl;
        return 0;
    }
    cv::Mat visualize = cv::Mat(map.rows, map.cols, CV_8UC3, cv::Scalar(255, 255, 255));

    auto worldYToImageRow = [&](float wy)->int{
        int row = (map.rows - 1) - static_cast<int>(wy * 20.0f);
        if(row < 0) row = 0;
        if(row >= map.rows) row = map.rows - 1;
        return row;
    };

    start.y() = static_cast<float>(worldYToImageRow(start.y())) * 0.05f;
    end.y()   = static_cast<float>(worldYToImageRow(end.y())) * 0.05f;

    YAstar astar(map.cols, map.rows, 0.05f, 0.f, 0.f);
    astar.setMap(map.cols, map.rows, map.data);
    astar.setMaskNumLayers(4);

    auto totalTimer = std::chrono::high_resolution_clock::now();

    benchTimeVoid("set cost map", [&](){
        astar.setCostField(1.f, [](float x){ return 0.5f / (0.1f + x); });
        astar.initCostMap();
    });

    std::vector<Eigen::Vector2f> path = benchTime("astar search", [&](){
        return astar.search(start, end);
    });

    YAstar::TMatrix<u_char> mapMatrix = astar.getMap();
    for(int i = 0; i < mapMatrix.rows(); i++){
        for(int j = 0; j < mapMatrix.cols(); j++){
            if(mapMatrix(i, j) == 0){
                visualize.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    std::cout << "path length: " << YAstar::getLength(path) << std::endl;

    if(path.empty()){
        return 0;
    }

    std::vector<Eigen::Vector2f> raw_astar_path = path;
    benchTimeVoid("simplify path", [&](){
        path = astar.simplifyPath(path, 0.1f);
        path = astar.simplifyPathHypot(path);
    });

    std::vector<cv::Point> wayPoints;
    for (auto &p : path){
        wayPoints.emplace_back(static_cast<int>(p.x() * 20), static_cast<int>(p.y() * 20));
    }
    cv::polylines(visualize, wayPoints, false, cv::Scalar(0, 128, 0), 1, cv::LINE_AA);

    MinimumSnap minimumSnap;
    minimumSnap.setOrder(6);
    minimumSnap.setMaxDx(3);
    minimumSnap.setDt(0.1);
    minimumSnap.setMap(map, 0.05f, 0, 0);
    minimumSnap.setTL(10);

    MinimumSnap::SolveInput input;
    input.setPath(path);
    input.setCollisionCheckIter(6);
    input.setMaxSpeed(3.f);
    input.setMaxAcc(1.f);
    input.setMaxCorridorRange(2.5f);
    input.setCorridorShrink(0.0f);
    input.setNormTime(true);
    input.setBackend(MinimumSnap::Backend::OSQPCorridor);
    input.setInitVel({0.f, 0.f});

    auto result = benchTime("minimum snap solve", [&](){
        return minimumSnap.solve(input);
    });

    double totalTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - totalTimer).count();

    std::cout << "solve result: success=" << result.success << " iter=" << result.iter
              << " time=" << result.time * 1e3 << "ms"
              << " corridor size=" << result.corridor.size()
              << " path size=" << result.path.size() << std::endl;

    wayPoints.clear();
    for(auto& p : result.path){
        cv::Point point(p.x() * 20, p.y() * 20);
        wayPoints.push_back(point);
    }
    cv::polylines(visualize, wayPoints, false, cv::Scalar(0, 0, 192), 1, cv::LINE_AA);

    if(!result.corridor.empty()){
        auto rects2 = SfcSquare::pointPair2Rects(result.corridor);
        for(size_t a = 0; a < rects2.size(); a++){
            auto rect = rects2[a];
            rect.x = rect.x * 20;
            rect.y = rect.y * 20;
            rect.width = rect.width * 20;
            rect.height = rect.height * 20;
            cv::rectangle(visualize, rect, cv::Scalar(255, 0, 0), 1);
        }
    }

    cv::Mat vis_large;
    cv::resize(visualize, vis_large, cv::Size(visualize.cols * 3, visualize.rows * 3), 0, 0, cv::INTER_NEAREST);
    if(!save_out.empty()){
        cv::imwrite(save_out, vis_large);
        std::cout << "✅ 已保存原版演示结果图: " << save_out << std::endl;
        if(save_out.find("tdt_orig_rmuc2025_demo.png") != std::string::npos){
            mkdir("/tmp/tdt_rmuc2025_eval", 0777);
            {
                std::ofstream ofs("/tmp/tdt_rmuc2025_eval/astar_raw.txt");
                for(const auto& p : raw_astar_path) ofs << p.x() << " " << p.y() << "\n";
            }
            {
                std::ofstream ofs("/tmp/tdt_rmuc2025_eval/waypoints.txt");
                for(const auto& p : path) ofs << p.x() << " " << p.y() << "\n";
            }
            {
                std::ofstream ofs("/tmp/tdt_rmuc2025_eval/smooth_traj.txt");
                for(const auto& p : result.path) ofs << p.x() << " " << p.y() << "\n";
            }
            {
                std::ofstream ofs("/tmp/tdt_rmuc2025_eval/corridors.txt");
                for(const auto& c : result.corridor) ofs << c[0] << " " << c[1] << " " << c[2] << " " << c[3] << "\n";
            }
        }
    }
    if(g_show_gui){
        cv::imshow("TDT-Nav-Kit Original Demo - path", vis_large);
        cv::waitKey(0);
    }

    bool err = false;
    for(auto& p : result.path){
        if(p.x() < 0 || p.x() >= map.cols * 0.05f || p.y() < 0 || p.y() >= map.rows * 0.05f){
            err = true;
            break;
        }
    }

    if(!err){
        pass++;
        float rate = static_cast<float>(pass) / (pass + fail) * 100;
        std::cout << "\033[32mCheck Pass  accuracy: " << pass << "/" << pass + fail
                  << "  rate: " << rate << "%  timeTotal:" << totalTime * 1e6 << " us \033[0m" << std::endl;
    }else{
        fail++;
        std::cout << "\033[31mError accuracy: " << pass << "/" << pass + fail
                  << "  rate: " << static_cast<float>(pass) / (pass + fail) * 100 << "%\033[0m" << std::endl;
    }
    return totalTime;
}

bool testKastar(){
    cv::Mat map = cv::imread(g_root_dir + "/images/rmuc2025.png", cv::IMREAD_GRAYSCALE);
    if(map.empty()){
        std::cout << "testKastar: 地图读取失败" << std::endl;
        return false;
    }
    KinodynamicAstar astar(map.cols, map.rows, 0.05f, 0.f, 0.f);
    astar.setMap(map.cols, map.rows, map.data);
    astar.config.maxNodes = 150000;
    astar.config.sampleTime = 0.01;
    Eigen::Vector2d start(25.2695, 14.14615), end(1.47266, 9.1749);
    start.y() = (map.rows - 1) * 0.05 - start.y();
    end.y() = (map.rows - 1) * 0.05 - end.y();
    Eigen::Vector2d velocity(0, 0), acceleration(0, 0);
    auto result = benchTime("kinodynamic astar search", [&](){
        return astar.search(start, velocity, acceleration, end);
    });
    std::vector<Eigen::Vector2f> path;
    for(const auto& sample : result.trajectory){
        path.push_back(sample.state.head<2>().cast<float>());
    }
    cv::Mat image;
    cv::cvtColor(map, image, cv::COLOR_GRAY2BGR);
    std::vector<cv::Point> curve;
    for(const auto& point : path){
        curve.emplace_back(point.x() / 0.05, point.y() / 0.05);
    }
    if(!curve.empty()){
        cv::polylines(image, curve, false, cv::Scalar(0, 0, 192), 1, cv::LINE_AA);
    }
    for(size_t i = 0; i < result.trajectory.size(); i += 100){
        const auto& state = result.trajectory[i].state;
        if(state.tail<2>().norm() < 1e-6) continue;
        Eigen::Vector2d tip = state.head<2>() + state.tail<2>().normalized() * 0.5;
        cv::arrowedLine(image, cv::Point(state.x() / 0.05, state.y() / 0.05),
            cv::Point(tip.x() / 0.05, tip.y() / 0.05), cv::Scalar(0, 160, 160), 1, cv::LINE_AA, 0, 0.35);
    }
    cv::circle(image, cv::Point(start.x() / 0.05, start.y() / 0.05), 4, cv::Scalar(0, 160, 0), -1);
    cv::circle(image, cv::Point(end.x() / 0.05, end.y() / 0.05), 4, cv::Scalar(255, 0, 0), -1);
    cv::resize(image, image, cv::Size(image.cols * 3, image.rows * 3), 0, 0, cv::INTER_NEAREST);
    cv::imwrite(g_root_dir + "/maps/tdt_orig_kastar_demo.png", image);
    if(g_show_gui){
        cv::imshow("TDT-Nav-Kit Kinodynamic A*", image);
        cv::waitKey(0);
    }
    return true;
}

int main(int argc, char** argv){
    for(int i = 1; i < argc; ++i){
        if(std::string(argv[i]) == "--gui"){
            g_show_gui = true;
        }
    }
    std::cout << "====================================================================\n"
              << "  🏆 【TDT-Nav-Kit 原版演示示例 (play)】RMUC 2024 / 2025 赛场规划测试\n"
              << "====================================================================" << std::endl;
    testKastar();
    // 1. RMUC 2024 赛场原版示例复现 (对标官方 README images/example.png)
    test({25.2695f, 14.14615f}, {1.47266f, 9.1749f}, "rmuc2024.png", g_root_dir + "/maps/tdt_orig_rmuc2024_demo.png");
    // 2. RMUC 2025 赛场原版跨全场对角规划复现
    test({25.2695f, 14.14615f}, {1.47266f, 9.1749f}, "rmuc2025.png", g_root_dir + "/maps/tdt_orig_rmuc2025_demo.png");
    test({2.5323f, 1.88688f}, {25.5733f, 13.5454f}, "rmuc2025.png", g_root_dir + "/maps/tdt_orig_rmuc2025_diag.png");
    test({21.5323f, 4.88688f}, {4.1733f, 14.1454f}, "rmuc2025.png");
    test({4.0f, 8.0f}, {7.0f, 0.95f}, "rmuc2025.png");
    test({19.2695f, 8.54615f}, {1.47266f, 9.1749f}, "rmuc2025.png");
    test({4.29f, 13.05f}, {23.7f, 0.95f}, "rmuc2025.png");

    float rate = static_cast<float>(pass) / (pass + fail) * 100;
    std::cout << "\033[32mCheck Pass  accuracy: " << pass << "/" << pass + fail << "  rate: " << rate << "%\033[0m" << std::endl;
    for(const auto& r : record){
        std::string name = r.first;
        double avg = std::get<1>(r.second);
        double max = std::get<2>(r.second);
        std::cout << "Function: " << name << " | Average: " << avg * 1e6 << " us | Max: " << max * 1e6 << " us" << std::endl;
    }
    return 0;
}
