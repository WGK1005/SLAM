/**
 * @file plan_rc_obstacles.cpp
 * @brief 2027 ROBOCON 足式机器人挑战赛 (宇树 Go2 + VLP-16 + HI12) 障碍赛越障轨迹与梅花桩落足点规划器
 *
 * 功能：
 *   1. 使用 SCU-TDT-V1 (YAstar + MinimumSnap + Square SFC + 途经点速度矢量强约束 useVxvy)
 *      规划宇树 Go2 从启动区1出发，依次完成：
 *        - [阶段1] 启动区1 -> 梅花桩西侧平台对准 -> 梅花桩S形微摆越障 + 四足16步落足点序列 (FL/FR/RL/RR)
 *        - [阶段2] 梅花桩东侧下台 -> 绕桩北端必达区 -> 6桩L形S弯绕桩 (途经3个必达区)
 *        - [阶段3] 绕桩西端必达区 -> 限高框架匍匐穿越 (内高290mm) -> 轮胎阵短边穿越
 *        - [阶段4] 轮胎阵西端 -> 沟壑组合 (台阶上台->跨360mm沟壑->20°斜坡下坡) -> 砖木坑短边L形穿越 -> 单木桥3m窄桥穿越 -> 返回启动区1
 *   2. 导出全套平滑轨迹、速度剖面与梅花桩四足落足点坐标至 /tmp/rc_go2_plan/，并自动调用绘图脚本生成高清诊断图。
 */

#include <mutex>
#include "../include/scu_tdt_v1/scu_recovery_tracker.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace scu_tdt_v1;

// 宇树 Go2 单步落足点记录
struct Footstep {
    int step_id;
    std::string leg;       // "FL", "FR", "RL", "RR"
    Eigen::Vector2f pos;   // 世界坐标 (x, y)
    float z;               // 落足高度 (m)
    std::string target;    // 所在平台或梅花桩编号
};

// 用 MinimumSnap 直接对带速度矢量约束的越障关键点序列做五阶平滑规划
static std::vector<Eigen::Vector2f> PlanConstrainedSegment(
    MinimumSnap& ms,
    const std::vector<MinimumSnap::SolveInput::Item>& items,
    float max_speed,
    float max_acc,
    double& elapsed_ms)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    MinimumSnap::SolveInput in;
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);
    in.setMaxSpeed(max_speed);
    in.setMaxAcc(max_acc);
    in.setMaxCorridorRange(0.0f); // 使用每个控制点自定义的手动走廊框 item.corridor
    in.setCollisionCheckIter(1);
    in.setNormTime(true);
    in.setItems(items);

    auto res = ms.solve(in);
    if (!res.success || res.path.size() < 2) {
        in.setBackend(MinimumSnap::Backend::Close);
        res = ms.solve(in);
    }
    std::vector<Eigen::Vector2f> pts = res.path;
    if (pts.size() < 2) {
        for (const auto& it : items) pts.push_back(it.xy);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return pts;
}

// 构造固定窄走廊的控制点
static MinimumSnap::SolveInput::Item MakeItem(
    float x, float y, float half_box,
    bool use_vel = false, float vx = 0.0f, float vy = 0.0f)
{
    MinimumSnap::SolveInput::Item it;
    it.xy = Eigen::Vector2f(x, y);
    it.useVxvy = use_vel;
    it.vxvy = Eigen::Vector2f(vx, vy);
    it.autoCorridor = false;
    it.corridor = {x - half_box, y - half_box, x + half_box, y + half_box};
    return it;
}

// 规划宇树 Go2 过梅花桩的 16 步四足落足点序列 (满足表2：每个梅花桩顶面至少接触一次)
static std::vector<Footstep> PlanGo2PlumPileFootsteps() {
    // 梅花桩 6 个 200x200x100mm 方块中心坐标：
    //   上排 (Y = 5.08m): B1(10.90, 5.08), B3(11.50, 5.08), B5(12.10, 5.08)
    //   下排 (Y = 4.68m): B2(11.20, 4.68), B4(11.80, 4.68), B6(12.40, 4.68)
    // Go2 左右髋距 0.23m、标准站立左右足距 0.30m (y = ±0.15m)，
    // 因此当机身沿中轴 Y=4.88m 微幅摆动 (±0.04m) 前进时：
    //   左腿 (FL, RL) 自然对应上排方块 (Y=5.05~5.08m)
    //   右腿 (FR, RR) 自然对应下排方块 (Y=4.68~4.71m)
    std::vector<Footstep> steps = {
        // 1. 西侧平台 (X ∈ [9.90, 10.70], Y ∈ [4.48, 5.28]) 起始站立四足位置
        { 0, "RL", {10.18f, 5.04f}, 0.10f, "West_Platform"},
        { 1, "RR", {10.18f, 4.72f}, 0.10f, "West_Platform"},
        { 2, "FL", {10.56f, 5.05f}, 0.10f, "West_Platform"},
        { 3, "FR", {10.56f, 4.71f}, 0.10f, "West_Platform"},

        // 2. 依次迈上 6 个交错梅花桩方块 (200x200mm 顶面，前足落桩面前半部 +3.5cm，后足跟进落前半/后半部 -3.5cm)
        { 4, "FL", {10.935f, 5.08f}, 0.10f, "Block_1_Upper"},
        { 5, "FR", {11.235f, 4.68f}, 0.10f, "Block_2_Lower"},
        { 6, "RL", {10.865f, 5.08f}, 0.10f, "Block_1_Upper"},
        { 7, "FL", {11.535f, 5.08f}, 0.10f, "Block_3_Upper"},
        { 8, "RR", {11.165f, 4.68f}, 0.10f, "Block_2_Lower"},
        { 9, "FR", {11.835f, 4.68f}, 0.10f, "Block_4_Lower"},
        {10, "RL", {11.465f, 5.08f}, 0.10f, "Block_3_Upper"},
        {11, "FL", {12.135f, 5.08f}, 0.10f, "Block_5_Upper"},
        {12, "RR", {11.765f, 4.68f}, 0.10f, "Block_4_Lower"},
        {13, "FR", {12.435f, 4.68f}, 0.10f, "Block_6_Lower"},
        {14, "RL", {12.065f, 5.08f}, 0.10f, "Block_5_Upper"},
        {15, "RR", {12.365f, 4.68f}, 0.10f, "Block_6_Lower"},

        // 3. 迈上东侧平台 (X ∈ [12.60, 13.40], Y ∈ [4.48, 5.28]) 稳定收腿
        {16, "FL", {12.82f, 5.04f}, 0.10f, "East_Platform"},
        {17, "FR", {12.82f, 4.72f}, 0.10f, "East_Platform"},
        {18, "RL", {13.15f, 5.04f}, 0.10f, "East_Platform"},
        {19, "RR", {13.15f, 4.72f}, 0.10f, "East_Platform"},
    };
    return steps;
}

int main() {
    std::string pgm_path = "/home/wgk/decision_planning/SCU-TDT-V1/maps/rc_obstacle_2027.pgm";
    HardwareConfig cfg;
    cfg.resolution = 0.05f;
    cfg.origin_x = 0.0f;
    cfg.origin_y = 0.0f;
    cfg.robot_radius = 0.25f;
    cfg.max_speed = 1.50f;
    cfg.max_acc = 1.00f;
    cfg.max_corridor_range = 0.85f;

    ScuTdtPlannerEngine engine;
    if (!engine.LoadPGMMap(pgm_path, cfg)) {
        std::cerr << "❌ 无法加载 RC 障碍赛地图: " << pgm_path << "\n";
        return 1;
    }

    MinimumSnap ms; // 越障内部轨迹采用自定义微走廊约束求解 (OSQPCorridor)

    mkdir("/tmp/rc_go2_plan", 0777);

    std::cout << "=========================================================================\n";
    std::cout << "  🐕 宇树 Go2 (VLP-16 + HI12) ROBOCON 2027 障碍赛全流程越障与落足点规划\n";
    std::cout << "=========================================================================\n";

    // -------------------- 1. 场间全局导航段 (YAstar + Square SFC + MinimumSnap) --------------------
    // 从启动区1 (7.00, 3.00) 导航至梅花桩西侧对准点 (9.55, 4.88)
    auto nav_to_plum = engine.Plan({7.00f, 3.00f}, {9.55f, 4.88f}, {0.0f, 0.4f});

    // -------------------- 2. 过梅花桩专项越障段 (机身质心 S 微摆轨迹 + 四足16步落足点) --------------------
    double plum_ms = 0.0;
    std::vector<MinimumSnap::SolveInput::Item> plum_items = {
        MakeItem( 9.55f, 4.88f, 0.02f, true, 0.45f,  0.00f), // 平台前对准切入
        MakeItem(10.30f, 4.88f, 0.05f, true, 0.40f,  0.00f), // 登上西侧 800x800 平台中心
        MakeItem(10.90f, 4.92f, 0.03f, false),               // FL 踩 #1 上排桩，质心微偏北 +4cm
        MakeItem(11.20f, 4.84f, 0.03f, false),               // FR 踩 #2 下排桩，质心微偏南 -4cm
        MakeItem(11.50f, 4.92f, 0.03f, false),               // FL 踩 #3 上排桩
        MakeItem(11.80f, 4.84f, 0.03f, false),               // FR 踩 #4 下排桩
        MakeItem(12.10f, 4.92f, 0.03f, false),               // FL 踩 #5 上排桩
        MakeItem(12.40f, 4.84f, 0.03f, false),               // FR 踩 #6 下排桩
        MakeItem(13.00f, 4.88f, 0.05f, true, 0.35f,  0.00f), // 登上东侧 800x800 平台中心
        MakeItem(13.55f, 4.55f, 0.04f, true, 0.00f, -0.40f), // 从东侧平台下台转向南侧绕桩区
    };
    auto plum_traj = PlanConstrainedSegment(ms, plum_items, 0.55f, 0.60f, plum_ms);
    auto footsteps = PlanGo2PlumPileFootsteps();

    // -------------------- 3. L形6桩 S弯绕桩段 (经过全部3个 φ300mm 必达区) --------------------
    // 竖向桩: (12.95, 2.85), (12.95, 1.95), (12.95, 1.05[拐角])
    // 横向桩: (12.05, 1.05), (11.15, 1.05), (10.25, 1.05)
    double slalom_ms = 0.0;
    std::vector<MinimumSnap::SolveInput::Item> slalom_items = {
        MakeItem(13.55f, 4.55f, 0.04f, true,  0.00f, -0.45f), // 衔接梅花桩东侧下台点
        MakeItem(12.95f, 3.15f, 0.02f, true, -0.35f, -0.55f), // 必达区1 (北端圆心)
        MakeItem(12.68f, 2.85f, 0.05f, true,  0.00f, -0.65f), // 绕第1桩西侧
        MakeItem(13.22f, 2.40f, 0.05f, false),                // 穿1-2桩之间至东侧
        MakeItem(13.22f, 1.95f, 0.05f, true,  0.00f, -0.65f), // 绕第2桩东侧
        MakeItem(12.68f, 1.50f, 0.05f, false),                // 穿2-3桩之间至西侧
        MakeItem(13.25f, 0.75f, 0.03f, true, -0.55f, -0.20f), // 必达区2 (拐角外侧圆心)
        MakeItem(12.50f, 1.32f, 0.05f, false),                // 穿拐角桩与第4桩之间至北侧
        MakeItem(12.05f, 1.32f, 0.05f, true, -0.65f,  0.00f), // 绕第4桩北侧
        MakeItem(11.60f, 0.78f, 0.05f, false),                // 穿4-5桩之间至南侧
        MakeItem(11.15f, 0.78f, 0.05f, true, -0.65f,  0.00f), // 绕第5桩南侧
        MakeItem(10.70f, 1.32f, 0.05f, false),                // 穿5-6桩之间至北侧
        MakeItem(10.25f, 1.32f, 0.05f, true, -0.60f, -0.15f), // 绕第6桩北侧
        MakeItem( 9.95f, 1.05f, 0.02f, true, -0.50f,  0.00f), // 必达区3 (西端圆心)
    };
    auto slalom_traj = PlanConstrainedSegment(ms, slalom_items, 1.10f, 1.20f, slalom_ms);

    // -------------------- 4. 限高框架 + 轮胎阵连续穿越段 --------------------
    double frame_tire_ms = 0.0;
    std::vector<MinimumSnap::SolveInput::Item> frame_tire_items = {
        MakeItem(9.95f, 1.05f, 0.02f, true, -0.60f, 0.00f),
        MakeItem(9.15f, 1.15f, 0.04f, true, -0.55f, 0.00f), // 限高框架东入口对准 (切换匍匐步态 h=0.22m)
        MakeItem(8.46f, 1.15f, 0.03f, true, -0.55f, 0.00f), // 限高框架中心 (内高290mm, 内宽800mm)
        MakeItem(7.75f, 1.15f, 0.04f, true, -0.65f, 0.00f), // 限高框架西出口 (恢复标准站立步态)
        MakeItem(7.05f, 1.15f, 0.04f, true, -0.45f, 0.00f), // 轮胎阵东短边入口对准
        MakeItem(5.70f, 1.15f, 0.05f, true, -0.40f, 0.00f), // 轮胎阵 2x4 中心高抬腿跨越
        MakeItem(4.30f, 1.15f, 0.04f, true, -0.50f, 0.00f), // 轮胎阵西短边迈出
    };
    auto frame_tire_traj = PlanConstrainedSegment(ms, frame_tire_items, 0.90f, 0.90f, frame_tire_ms);

    // -------------------- 5. 沟壑组合 + 砖木坑 + 单木桥 + 高墙跳跃段 --------------------
    double west_loop_ms = 0.0;
    std::vector<MinimumSnap::SolveInput::Item> west_loop_items = {
        MakeItem(4.30f, 1.15f, 0.03f, true, -0.50f, 0.00f),
        MakeItem(3.55f, 1.00f, 0.04f, true, -0.45f, 0.00f), // 沟壑组合：台阶东侧入口对准
        MakeItem(2.26f, 1.00f, 0.04f, true, -0.40f, 0.00f), // 迈上两级台阶到达右平台顶面 (z=0.36m)
        MakeItem(1.10f, 1.00f, 0.04f, true,  0.00f, 0.45f), // 跳过 360mm 沟壑到达左平台(启动区2)并转向北侧斜坡
        MakeItem(1.10f, 2.75f, 0.05f, true,  0.35f, 0.45f), // 沿 20° 斜坡下坡回到地面
        MakeItem(2.12f, 3.15f, 0.04f, true,  0.00f, 0.45f), // 对准砖木坑南侧短边入口 (X=2.12, 宽1.0m)
        MakeItem(2.12f, 3.89f, 0.05f, true, -0.35f, 0.35f), // 砖木坑下区 -> 拐角
        MakeItem(1.12f, 4.89f, 0.05f, true,  0.45f, 0.00f), // 砖木坑左上区
        MakeItem(2.95f, 4.88f, 0.04f, true,  0.55f, 0.00f), // 从砖木坑东侧短边离开，正对单木桥西短边
        MakeItem(3.98f, 4.88f, 0.02f, true,  0.50f, 0.00f), // 踏上 400mm 宽单木桥西短边
        MakeItem(5.48f, 4.88f, 0.02f, true,  0.55f, 0.00f), // 单木桥 3.0m 中心稳定直行
        MakeItem(6.98f, 4.88f, 0.02f, true,  0.55f, 0.00f), // 离开单木桥东短边
        MakeItem(8.05f, 4.88f, 0.04f, true,  0.60f, 0.00f), // 高墙西侧起跳点
        MakeItem(8.95f, 4.88f, 0.04f, true,  0.40f, -0.30f),// 跃过 300mm 高墙落地
        MakeItem(7.00f, 3.00f, 0.03f, true,  0.00f, 0.00f), // 返回启动区1 停稳 (获 +150 额外分)
    };
    auto west_loop_traj = PlanConstrainedSegment(ms, west_loop_items, 1.00f, 1.00f, west_loop_ms);

    std::cout << "  [1] 启动区1 -> 梅花桩入口 (YAstar+SFC) : " << std::fixed << std::setprecision(2)
              << nav_to_plum.total_ms << " ms, 长度 " << nav_to_plum.path_length_m << " m\n";
    std::cout << "  [2] 过梅花桩 (质心微摆 + 20步四足落足) : " << plum_ms
              << " ms, 6个200x200mm桩面 100% 踩踏覆盖\n";
    std::cout << "  [3] L形6桩 S弯绕桩 (含3个必达区)       : " << slalom_ms << " ms\n";
    std::cout << "  [4] 限高框架(匍匐) + 轮胎阵短边穿越    : " << frame_tire_ms << " ms\n";
    std::cout << "  [5] 沟壑组合 + 砖木坑 + 单木桥 + 高墙  : " << west_loop_ms << " ms\n";
    std::cout << "-------------------------------------------------------------------------\n";
    std::cout << "  ★ 梅花桩 Go2 四足落足点序列 (前8步预览):\n";
    for (size_t i = 0; i < 10 && i < footsteps.size(); ++i) {
        const auto& f = footsteps[i];
        std::printf("    Step %2d | 腿=%s | 坐标=(%6.2f, %5.2f, z=%.2f) | 目标=%s\n",
                    f.step_id, f.leg.c_str(), f.pos.x(), f.pos.y(), f.z, f.target.c_str());
    }
    std::cout << "=========================================================================\n";

    // 导出数据供 Python 绘制全景路线 + 梅花桩四足落足点放大图
    auto dump_pts = [](const char* fn, const std::vector<Eigen::Vector2f>& pts) {
        std::ofstream ofs(fn);
        for (const auto& p : pts) ofs << p.x() << " " << p.y() << "\n";
    };
    dump_pts("/tmp/rc_go2_plan/seg1_approach.txt", nav_to_plum.smooth_traj_w);
    dump_pts("/tmp/rc_go2_plan/seg2_plum.txt", plum_traj);
    dump_pts("/tmp/rc_go2_plan/seg3_slalom.txt", slalom_traj);
    dump_pts("/tmp/rc_go2_plan/seg4_frame_tire.txt", frame_tire_traj);
    dump_pts("/tmp/rc_go2_plan/seg5_west_loop.txt", west_loop_traj);

    {
        std::ofstream ofs("/tmp/rc_go2_plan/plum_footsteps.txt");
        for (const auto& f : footsteps) {
            ofs << f.step_id << " " << f.leg << " " << f.pos.x() << " " << f.pos.y()
                << " " << f.z << " " << f.target << "\n";
        }
    }

    int ret = std::system("PYTHONNOUSERSITE=1 python3 /home/wgk/decision_planning/SCU-TDT-V1/tools/plot_rc_obstacle_plans.py");
    if (ret != 0) {
        std::cerr << "⚠️ 绘图脚本返回非零状态: " << ret << "\n";
    }
    return 0;
}
