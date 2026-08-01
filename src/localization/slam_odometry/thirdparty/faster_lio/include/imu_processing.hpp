#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

/**
 * @file imu_processing.hpp
 * @brief IMU 数据处理与点云去畸变模块
 *
 * 该模块是 Faster-LIO 的核心组件之一，负责：
 * - IMU 数据初始化：估计重力方向、陀螺仪零偏、加速度计噪声协方差
 * - IMU 状态预测：利用 IMU 数据进行 ESKF 状态预测
 * - 点云去畸变（运动补偿）：利用 IMU 位姿序列补偿 LiDAR 扫描期间的运动畸变
 *
 * 算法流程：
 * 1. IMUInit: 收集前 MAX_INI_COUNT 帧的 IMU 数据，估计初始状态
 * 2. UndistortPcl: 利用 IMU 位姿序列对点云进行去畸变
 *    - 按时间排序点云（根据 curvature 字段存储的时间偏移）
 *    - 利用 IMU 数据进行 ESKF 预测，构建位姿序列
 *    - 对每个点，根据其时间偏移插值对应的 IMU 位姿
 *    - 将点从扫描时刻坐标系转换到扫描结束时刻坐标系
 *
 * 坐标系说明：
 * - IMU 坐标系：与 IMU 传感器固连，X-前 Y-左 Z-上
 * - LiDAR 坐标系：与 LiDAR 传感器固连
 * - 世界坐标系：初始 IMU 坐标系，Z 轴与重力方向对齐
 * - 变换链：p_world = R_imu * (R_offset * p_lidar + t_offset) + t_imu
 *
 * @see common::MeasureGroup（输入数据结构）
 * @see esekfom::esekf（误差状态卡尔曼滤波器）
 * @see state_ikfom（状态向量定义）
 */

// 第三方库
#include <glog/logging.h>  // Google 日志库
#include <cmath>           // 数学函数
#include <cassert>         // 断言
#include <deque>           // 双端队列
#include <fstream>         // 文件流
#include <memory>          // 智能指针

// Faster-LIO 内部头文件
#include "common_lib.h"   // 通用类型和工具
#include "so3_math.h"     // SO(3) 流形数学工具
#include "use-ikfom.hpp"  // IKFoM 滤波器定义
#include "utils.h"        // 工具函数

namespace faster_lio {

/**
 * @brief IMU 初始化最大帧数
 *
 * 在系统启动时，需要收集至少 MAX_INI_COUNT 帧 IMU 数据来估计：
 * - 重力方向（通过平均加速度）
 * - 陀螺仪零偏（通过平均角速度）
 * - 加速度计和陀螺仪的噪声协方差
 *
 * 默认值 20 帧，约 0.2~0.4 秒（取决于 IMU 采样率）
 */
constexpr int MAX_INI_COUNT = 20;

/**
 * @brief 获取 IMU 样本的时间戳
 *
 * @param imu IMU 样本指针
 * @return double 时间戳（秒）
 *
 * 辅助函数，用于从 ImuSamplePtr 中提取时间戳
 */
inline double imuTimestamp(const ImuSamplePtr& imu) {
    return imu->timestamp;
}

/**
 * @brief 点云时间排序比较函数
 *
 * @param x 第一个点
 * @param y 第二个点
 * @return true 如果 x 的时间偏移小于 y
 * @return false 否则
 *
 * 用于 std::sort 对点云按时间偏移排序。
 * curvature 字段存储的是时间偏移（单位 ms），
 * 排序后点云按扫描时间先后顺序排列，便于后续去畸变处理。
 */
bool time_list(const PointType& x, const PointType& y) {
    return x.curvature < y.curvature;
}

/**
 * @brief IMU 处理器类
 *
 * 核心功能：
 * 1. IMU 初始化（IMUInit）：估计初始重力、零偏、噪声协方差
 * 2. 点云去畸变（UndistortPcl）：利用 IMU 位姿序列补偿运动畸变
 * 3. 状态预测：通过 ESKF 预测 IMU 状态
 *
 * 使用方式：
 * @code
 * ImuProcess imu_process;
 * imu_process.SetExtrinsic(t_lidar_imu, R_lidar_imu);  // 设置 LiDAR-IMU 外参
 * imu_process.SetGyrCov(cov_gyr);                       // 设置陀螺仪噪声协方差
 * imu_process.SetAccCov(cov_acc);                       // 设置加速度计噪声协方差
 * imu_process.Process(meas, kf_state, pcl_un);          // 处理一帧数据
 * @endcode
 */
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen 内存对齐宏

    /**
     * @brief 构造函数
     *
     * 初始化所有成员变量：
     * - 过程噪声协方差 Q_
     * - 加速度计/陀螺仪噪声协方差
     * - 零偏噪声协方差
     * - 平均加速度/角速度
     * - LiDAR-IMU 外参（默认单位旋转、零平移）
     * - 状态标志（首帧、需要初始化）
     */
    ImuProcess();

    /**
     * @brief 析构函数
     */
    ~ImuProcess();

    /**
     * @brief 重置 IMU 处理器状态
     *
     * 重置所有状态变量，用于重新初始化或异常恢复：
     * - 平均加速度/角速度
     * - 初始化标志
     * - IMU 位姿序列
     * - 上一帧 IMU 数据
     */
    void Reset();

    /**
     * @brief 设置 LiDAR-IMU 外参
     *
     * @param transl LiDAR 相对于 IMU 的平移（IMU 坐标系下）
     * @param rot LiDAR 相对于 IMU 的旋转（IMU 坐标系下）
     *
     * 外参定义：p_imu = R * p_lidar + t
     * 其中 p_imu 是点在 IMU 坐标系下的坐标，p_lidar 是点在 LiDAR 坐标系下的坐标
     */
    void SetExtrinsic(const common::V3D& transl, const common::M3D& rot);

    /**
     * @brief 设置陀螺仪测量噪声协方差缩放因子
     *
     * @param scaler 三轴缩放因子
     *
     * 用于调整陀螺仪测量噪声的协方差矩阵
     */
    void SetGyrCov(const common::V3D& scaler);

    /**
     * @brief 设置加速度计测量噪声协方差缩放因子
     *
     * @param scaler 三轴缩放因子
     *
     * 用于调整加速度计测量噪声的协方差矩阵
     */
    void SetAccCov(const common::V3D& scaler);

    /**
     * @brief 设置陀螺仪零偏随机游走协方差
     *
     * @param b_g 三轴零偏协方差
     *
     * 陀螺仪零偏被视为随机游走过程，该参数控制其扩散速度
     */
    void SetGyrBiasCov(const common::V3D& b_g);

    /**
     * @brief 设置加速度计零偏随机游走协方差
     *
     * @param b_a 三轴零偏协方差
     *
     * 加速度计零偏被视为随机游走过程，该参数控制其扩散速度
     */
    void SetAccBiasCov(const common::V3D& b_a);

    /**
     * @brief 主处理函数：IMU 初始化或点云去畸变
     *
     * @param meas 测量数据组（包含 LiDAR 点云和同步的 IMU 数据）
     * @param kf_state ESKF 状态（输入/输出）
     * @param cur_pcl_un_ 去畸变后的点云输出
     *
     * 处理逻辑：
     * - 若 IMU 未初始化（imu_need_init_ = true）：调用 IMUInit 进行初始化
     * - 若 IMU 已初始化：调用 UndistortPcl 进行点云去畸变
     *
     * @note 初始化需要至少 MAX_INI_COUNT 帧 IMU 数据
     */
    void Process(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                 PointCloudType::Ptr cur_pcl_un_);

    // ==================== 公共成员变量 ====================

    /**
     * @brief 过程噪声协方差矩阵（12x12）
     *
     * 布局（按 input_ikfom 定义）：
     * - [0:3, 0:3]: 陀螺仪测量噪声
     * - [3:6, 3:6]: 加速度计测量噪声
     * - [6:9, 6:9]: 陀螺仪零偏随机游走
     * - [9:12, 9:12]: 加速度计零偏随机游走
     */
    Eigen::Matrix<double, 12, 12> Q_;

    common::V3D cov_acc_;       ///< 加速度计噪声协方差（在线估计）
    common::V3D cov_gyr_;       ///< 陀螺仪噪声协方差（在线估计）
    common::V3D cov_acc_scale_; ///< 加速度计噪声协方差缩放因子（配置）
    common::V3D cov_gyr_scale_; ///< 陀螺仪噪声协方差缩放因子（配置）
    common::V3D cov_bias_gyr_;  ///< 陀螺仪零偏协方差（配置）
    common::V3D cov_bias_acc_;  ///< 加速度计零偏协方差（配置）

private:
    /**
     * @brief IMU 初始化
     *
     * @param meas 测量数据组
     * @param kf_state ESKF 状态（输出初始化后的状态）
     * @param N 初始化计数器（引用，用于累积统计）
     *
     * 初始化流程：
     * 1. 首帧处理：重置状态，初始化均值
     * 2. 累积统计：计算加速度/角速度的均值和协方差
     * 3. 状态设置：
     *    - 重力方向：g = -mean_acc / |mean_acc| * G_m_s2
     *    - 陀螺仪零偏：bg = mean_gyr
     *    - LiDAR-IMU 外参
     * 4. 协方差初始化：设置初始协方差矩阵
     *
     * 统计公式（Welford 在线算法）：
     * - 均值更新：mean += (x - mean) / N
     * - 协方差更新：cov = cov * (N-1)/N + (x-mean)^2 * (N-1)/(N^2)
     */
    void IMUInit(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);

    /**
     * @brief 点云去畸变（运动补偿）
     *
     * @param meas 测量数据组
     * @param kf_state ESKF 状态（输入当前状态，输出预测后的状态）
     * @param pcl_out 去畸变后的点云（输出）
     *
     * 去畸变原理：
     * LiDAR 扫描一帧点云需要时间（如 100ms），在此期间传感器在运动，
     * 导致点云产生畸变。去畸变将每个点从"扫描时刻"的坐标系转换到
     * "扫描结束时刻"的统一坐标系。
     *
     * 处理流程：
     * 1. 准备 IMU 数据：将上一帧最后一个 IMU 加入序列
     * 2. 按时间排序点云（根据 curvature 字段）
     * 3. ESKF 预测：利用 IMU 数据进行状态预测，构建位姿序列
     * 4. 点云补偿：对每个点，根据其时间偏移找到对应的 IMU 位姿，
     *    计算从扫描时刻到扫描结束时刻的变换
     *
     * 坐标变换公式：
     * p_compensate = R_offset^T * (R_end^T * (R_i * (R_offset * p + t_offset) + T_ei) - t_offset)
     *
     * 其中：
     * - R_offset, t_offset: LiDAR→IMU 外参
     * - R_end, t_end: 扫描结束时刻的 IMU 位姿
     * - R_i, t_i: 点扫描时刻的 IMU 位姿（通过插值得到）
     * - T_ei = t_i + v_i * dt + 0.5 * a_i * dt^2 - t_end
     */
    void UndistortPcl(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                      PointCloudType& pcl_out);

    // ==================== 私有成员变量 ====================

    ImuSamplePtr last_imu_;              ///< 上一帧最后一个 IMU 样本（用于连接相邻帧）
    std::vector<common::Pose6D> IMUpose_; ///< IMU 位姿序列（用于点云去畸变插值）
    std::vector<common::M3D> v_rot_pcl_;  ///< 点云旋转序列（未使用）

    common::M3D Lidar_R_wrt_IMU_;  ///< LiDAR 相对于 IMU 的旋转矩阵
    common::V3D Lidar_T_wrt_IMU_;  ///< LiDAR 相对于 IMU 的平移向量

    common::V3D mean_acc_;  ///< 加速度计测量均值（用于重力估计）
    common::V3D mean_gyr_;  ///< 陀螺仪测量均值（用于零偏估计）

    common::V3D angvel_last_;  ///< 上一帧角速度（用于状态传播）
    common::V3D acc_s_last_;   ///< 上一帧加速度（世界坐标系，用于状态传播）

    double last_lidar_end_time_ = 0;  ///< 上一帧 LiDAR 结束时间戳
    int init_iter_num_ = 1;           ///< 初始化迭代计数器
    bool b_first_frame_ = true;       ///< 是否为第一帧标志
    bool imu_need_init_ = true;       ///< IMU 是否需要初始化标志
};

// =============================================================================
// 成员函数实现
// =============================================================================

/**
 * @brief 构造函数实现
 *
 * 初始化所有成员变量为默认值：
 * - 过程噪声协方差 Q_：通过 process_noise_cov() 获取默认值
 * - 加速度计/陀螺仪噪声：0.1
 * - 零偏噪声：0.0001
 * - 平均加速度：(0, 0, -1.0)（假设静止时 Z 轴朝下）
 * - 平均角速度：(0, 0, 0)
 * - LiDAR-IMU 外参：零平移、单位旋转
 */
ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_ = process_noise_cov();  // 获取默认过程噪声协方差

    // 初始化噪声协方差（默认值，后续会被配置覆盖）
    cov_acc_ = common::V3D(0.1, 0.1, 0.1);
    cov_gyr_ = common::V3D(0.1, 0.1, 0.1);
    cov_bias_gyr_ = common::V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = common::V3D(0.0001, 0.0001, 0.0001);

    // 初始化均值（假设 IMU 静止放置，Z 轴朝下）
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);

    angvel_last_ = common::Zero3d;
    Lidar_T_wrt_IMU_ = common::Zero3d;
    Lidar_R_wrt_IMU_ = common::Eye3d;
    last_imu_ = std::make_shared<ImuSample>();
}

/**
 * @brief 析构函数实现
 */
ImuProcess::~ImuProcess() = default;

/**
 * @brief 重置 IMU 处理器状态
 *
 * 将所有状态变量重置为初始值，用于重新初始化
 */
void ImuProcess::Reset() {
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    imu_need_init_ = true;
    init_iter_num_ = 1;
    last_imu_ = std::make_shared<ImuSample>();
    IMUpose_.clear();
}

/**
 * @brief 设置 LiDAR-IMU 外参
 */
void ImuProcess::SetExtrinsic(const common::V3D& transl, const common::M3D& rot) {
    Lidar_T_wrt_IMU_ = transl;
    Lidar_R_wrt_IMU_ = rot;
}

/**
 * @brief 设置陀螺仪测量噪声协方差缩放因子
 */
void ImuProcess::SetGyrCov(const common::V3D& scaler) {
    cov_gyr_scale_ = scaler;
}

/**
 * @brief 设置加速度计测量噪声协方差缩放因子
 */
void ImuProcess::SetAccCov(const common::V3D& scaler) {
    cov_acc_scale_ = scaler;
}

/**
 * @brief 设置陀螺仪零偏随机游走协方差
 */
void ImuProcess::SetGyrBiasCov(const common::V3D& b_g) {
    cov_bias_gyr_ = b_g;
}

/**
 * @brief 设置加速度计零偏随机游走协方差
 */
void ImuProcess::SetAccBiasCov(const common::V3D& b_a) {
    cov_bias_acc_ = b_a;
}

/**
 * @brief IMU 初始化实现
 *
 * 详细流程：
 * 1. 首帧处理：
    *    - 重置所有状态
    *    - 使用第一组 IMU 数据初始化均值
 * 2. 在线统计（Welford 算法）：
    *    - 遍历所有 IMU 样本
    *    - 更新加速度/角速度均值
    *    - 更新加速度/角速度协方差
 * 3. 状态初始化：
    *    - 重力方向：通过平均加速度估计，归一化后乘以重力加速度
    *    - 陀螺仪零偏：平均角速度
    *    - LiDAR-IMU 外参
 * 4. 协方差初始化：
    *    - 设置初始协方差矩阵对角线元素
    *    - 位置/旋转协方差：0.00001
    *    - 外参平移协方差：0.0001
    *    - 外参旋转协方差：0.001
    *    - 速度协方差：0.00001
 *
 * @note 该函数会被连续调用多次（每帧一次），直到 init_iter_num_ > MAX_INI_COUNT
 */
void ImuProcess::IMUInit(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                         int& N) {
    common::V3D cur_acc, cur_gyr;

    // 首帧处理：重置状态并初始化均值
    if (b_first_frame_) {
        Reset();
        N = 1;  // 重置计数器
        b_first_frame_ = false;

        // 使用第一组 IMU 数据初始化均值
        const auto& imu_acc = meas.imu_.front()->linear_acceleration;
        const auto& gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }

    // 遍历所有 IMU 样本，更新统计量
    for (const auto& imu : meas.imu_) {
        const auto& imu_acc = imu->linear_acceleration;
        const auto& gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        // Welford 在线均值更新公式：mean += (x - mean) / N
        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        // Welford 在线协方差更新公式
        cov_acc_ = cov_acc_ * (N - 1.0) / N +
                   (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ = cov_gyr_ * (N - 1.0) / N +
                   (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }

    // 初始化 ESKF 状态
    state_ikfom init_state = kf_state.get_x();

    // 重力方向：与平均加速度方向相反，归一化后乘以重力加速度
    // 假设静止时加速度计测量值主要为重力加速度
    init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * common::G_m_s2);

    // 陀螺仪零偏：平均角速度（假设静止时角速度应为零）
    init_state.bg = mean_gyr_;

    // LiDAR-IMU 外参
    init_state.offset_T_L_I = Lidar_T_wrt_IMU_;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU_;

    // 应用初始状态
    kf_state.change_x(init_state);

    // 初始化协方差矩阵
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    // 位置协方差（索引 6,7,8）
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    // 旋转协方差（索引 9,10,11）
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    // 外参平移协方差（索引 15,16,17）
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    // 外参旋转协方差（索引 18,19,20）
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    // 速度协方差（索引 21,22）
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);

    // 保存最后一个 IMU 样本，用于下一帧连接
    last_imu_ = meas.imu_.back();
}

/**
 * @brief 点云去畸变实现
 *
 * 详细流程：
 * 1. 准备 IMU 数据：
 *    - 将上一帧最后一个 IMU 加入当前序列头部
 *    - 确定 IMU 时间范围 [imu_beg_time, imu_end_time]
 *    - 确定点云时间范围 [pcl_beg_time, pcl_end_time]
 *
 * 2. 点云排序：
 *    - 拷贝点云数据
 *    - 按 curvature 字段（时间偏移，单位 ms）排序
 *
 * 3. ESKF 预测与位姿序列构建：
 *    - 初始化 IMU 位姿序列，加入起始位姿
 *    - 遍历 IMU 数据对，进行 ESKF 预测
 *    - 计算补偿后的角速度和加速度
 *    - 将位姿加入序列
 *
 * 4. 点云时间外推：
 *    - 若点云结束时间晚于 IMU 结束时间，进行外推预测
 *
 * 5. 点云补偿：
 *    - 从最后一个点开始向前遍历
 *    - 对每个点，找到对应时间区间的 IMU 位姿
 *    - 计算从扫描时刻到扫描结束时刻的坐标变换
 *    - 更新点坐标
 *
 * 坐标变换详细推导：
 * 目标：将点 p 从"扫描时刻"转换到"扫描结束时刻"
 *
 * 1. 扫描时刻（t_i）：
 *    p_imu_i = R_offset * p_lidar + t_offset
 *    p_world_i = R_i * p_imu_i + t_i
 *
 * 2. 扫描结束时刻（t_end）：
 *    p_imu_end = R_offset * p_lidar' + t_offset
 *    p_world_end = R_end * p_imu_end + t_end
 *
 * 3. 假设世界坐标不变（p_world_i = p_world_end）：
 *    R_i * (R_offset * p + t_offset) + t_i = R_end * (R_offset * p' + t_offset) + t_end
 *
 * 4. 解出 p'（补偿后的点坐标）：
 *    p' = R_offset^T * (R_end^T * (R_i * (R_offset * p + t_offset) + T_ei) - t_offset)
 *
 * 其中 T_ei = t_i + v_i * dt + 0.5 * a_i * dt^2 - t_end
 * 考虑了 IMU 在 dt 时间内的运动（匀速加速度模型）
 */
void ImuProcess::UndistortPcl(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                              PointCloudType& pcl_out) {
    // 准备 IMU 数据：将上一帧最后一个 IMU 加入序列头部，保证时间连续性
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);

    // 确定时间范围
    const double imu_beg_time = imuTimestamp(v_imu.front());
    const double imu_end_time = imuTimestamp(v_imu.back());
    const double pcl_beg_time = meas.lidar_bag_time_;
    const double pcl_end_time = meas.lidar_end_time_;

    // 拷贝并排序点云
    pcl_out = *(meas.lidar_);
    // 按时间偏移排序（curvature 字段存储时间偏移，单位 ms）
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    // 获取当前 ESKF 状态
    state_ikfom imu_state = kf_state.get_x();

    // 初始化 IMU 位姿序列
    IMUpose_.clear();
    // 加入起始位姿（时间偏移为 0）
    IMUpose_.push_back(common::set_pose6d(0.0, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                          imu_state.rot.toRotationMatrix()));

    // 临时变量
    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    common::M3D R_imu;
    double dt = 0;

    input_ikfom in;
    // 遍历 IMU 数据对（每对构成一个时间区间）
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto head = *(it_imu);
        auto tail = *(it_imu + 1);

        // 跳过早于上一帧 LiDAR 结束时间的 IMU 数据
        if (imuTimestamp(tail) < last_lidar_end_time_) {
            continue;
        }

        // 计算区间内的平均角速度和平均加速度
        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        // 加速度归一化：将加速度计测量值转换为实际加速度（单位：m/s^2）
        acc_avr = acc_avr * common::G_m_s2 / mean_acc_.norm();

        // 计算时间间隔 dt
        if (imuTimestamp(head) < last_lidar_end_time_) {
            // 若区间头部早于上一帧结束时间，只计算重叠部分
            dt = imuTimestamp(tail) - last_lidar_end_time_;
        } else {
            // 正常情况：使用完整区间
            dt = imuTimestamp(tail) - imuTimestamp(head);
        }

        // 设置 ESKF 输入
        in.acc = acc_avr;
        in.gyro = angvel_avr;

        // 更新过程噪声协方差
        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;      // 陀螺仪噪声
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;      // 加速度计噪声
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_; // 陀螺仪零偏噪声
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_; // 加速度计零偏噪声

        // ESKF 预测：利用 IMU 数据预测状态
        kf_state.predict(dt, Q_, in);

        // 获取预测后的状态
        imu_state = kf_state.get_x();

        // 计算补偿后的角速度（减去零偏）
        angvel_last_ = angvel_avr - imu_state.bg;

        // 计算世界坐标系下的加速度
        acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav[i];  // 加上重力分量
        }

        // 计算时间偏移（相对于点云起始时刻）
        const double offs_t = imuTimestamp(tail) - pcl_beg_time;

        // 将当前位姿加入序列
        IMUpose_.emplace_back(common::set_pose6d(offs_t, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                                 imu_state.rot.toRotationMatrix()));
    }

    // 点云时间外推：若点云结束时间晚于 IMU 结束时间，进行外推预测
    const double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q_, in);

    // 更新状态
    imu_state = kf_state.get_x();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    // 若点云为空，直接返回
    if (pcl_out.points.empty()) {
        return;
    }

    // ==================== 点云补偿 ====================
    // 从最后一个点开始向前遍历（因为点云已按时间排序，后面的点时间更晚）
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose_.end() - 1; it_kp != IMUpose_.begin(); it_kp--) {
        auto head = it_kp - 1;  // 区间头部（较早时刻）
        auto tail = it_kp;      // 区间尾部（较晚时刻）

        // 从位姿序列中提取状态
        R_imu = common::MatFromArray(head->rot);    // 头部旋转矩阵
        vel_imu = common::VecFromArray(head->vel);  // 头部速度
        pos_imu = common::VecFromArray(head->pos);  // 头部位置
        acc_imu = common::VecFromArray(tail->acc);  // 尾部加速度
        angvel_avr = common::VecFromArray(tail->gyr); // 尾部角速度

        // 处理该时间区间内的所有点
        // 点的 curvature 字段存储时间偏移（单位 ms），除以 1000 转换为秒
        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            // 计算点相对于区间头部的时间差
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            // 计算点扫描时刻的旋转矩阵（通过指数映射）
            // R_i = R_head * Exp(angvel_avr * dt)
            common::M3D R_i(R_imu * Exp(angvel_avr, dt));

            // 点的原始坐标（LiDAR 坐标系）
            common::V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);

            // 计算从扫描时刻到扫描结束时刻的平移差
            // T_ei = pos_head + vel_head * dt + 0.5 * acc * dt^2 - pos_end
            common::V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);

            // 坐标变换：将点从扫描时刻转换到扫描结束时刻
            // 完整变换链：
            // 1. p_imu_i = R_offset * p_lidar + t_offset
            // 2. p_world_i = R_i * p_imu_i + pos_head
            // 3. p_world_end = R_end * p_imu_end + pos_end
            // 4. 令 p_world_i = p_world_end，解出 p_lidar'
            common::V3D p_compensate =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                 imu_state.offset_T_L_I);

            // 更新点坐标
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            // 若到达点云开头，退出循环
            if (it_pcl == pcl_out.points.begin()) {
                break;
            }
        }
    }
}

/**
 * @brief 主处理函数实现
 *
 * 处理逻辑：
 * 1. 检查 IMU 数据是否为空
 * 2. 检查 LiDAR 数据是否有效
 * 3. 若 IMU 未初始化：
 *    - 调用 IMUInit 进行初始化
 *    - 累积初始化计数器
 *    - 若达到 MAX_INI_COUNT，完成初始化并输出日志
 * 4. 若 IMU 已初始化：
 *    - 调用 UndistortPcl 进行点云去畸变
 *
 * @note 初始化阶段不会进行点云去畸变，直接返回
 */
void ImuProcess::Process(const common::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                         PointCloudType::Ptr cur_pcl_un_) {
    // 检查 IMU 数据是否为空
    if (meas.imu_.empty()) {
        return;
    }

    // 断言 LiDAR 数据有效
    assert(meas.lidar_ != nullptr);

    // IMU 初始化阶段
    if (imu_need_init_) {
        // 调用 IMU 初始化
        IMUInit(meas, kf_state, init_iter_num_);

        // 保持初始化标志（会在达到 MAX_INI_COUNT 后清除）
        imu_need_init_ = true;

        // 保存最后一个 IMU 样本
        last_imu_ = meas.imu_.back();

        // 检查是否达到最大初始化帧数
        if (init_iter_num_ > MAX_INI_COUNT) {
            // 根据重力加速度调整加速度计协方差
            cov_acc_ *= pow(common::G_m_s2 / mean_acc_.norm(), 2);

            // 完成初始化
            imu_need_init_ = false;

            // 使用配置的噪声协方差覆盖在线估计值
            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;

            // 输出初始化结果日志
            const state_ikfom done_state = kf_state.get_x();
            LOG(INFO) << "[SLAM][Core][IMUInit] Done init_iter_num=" << init_iter_num_
                      << " mean_acc=(" << mean_acc_.transpose() << ")"
                      << " mean_gyr=(" << mean_gyr_.transpose() << ")"
                      << " grav=(" << done_state.grav[0] << " " << done_state.grav[1] << " "
                      << done_state.grav[2] << ")"
                      << " bg=(" << done_state.bg.transpose() << ")"
                      << " last_imu_ts=" << imuTimestamp(last_imu_);
        }

        // 初始化阶段不处理点云，直接返回
        return;
    }

    // IMU 已初始化：进行点云去畸变
    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, *cur_pcl_un_); }, "Undistort Pcl");
}

}  // namespace faster_lio

#endif