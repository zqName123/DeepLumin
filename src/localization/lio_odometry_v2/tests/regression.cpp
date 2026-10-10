#include <iostream>
#include <stdexcept>
#include <Eigen/Eigenvalues>
#include <pcl/conversions.h>
#include "lio_odometry/core/laser_odometry_core.h"
using namespace slam_odometry;
void require(bool ok, const char* text) { if (!ok) throw std::runtime_error(text); }
IMUPtr imu(double t, const Vec3d& a, const Vec3d& w = Vec3d::Zero()) {
    auto p = std::make_shared<IMU>(); p->timestamp=t;
    p->linear_acceleration=a; p->angular_velocity=w; return p;
}
PointType point(float x,float y,float z,double t) {
    PointType p; p.x=x;p.y=y;p.z=z;p.intensity=1;p.time=t;return p;
}
void mathChecks() {
    for (const Vec3d& g : {Vec3d(0,0,-9.809),Vec3d(1,2,-9),Vec3d(-9.809,0,0)}) {
        S2 origin(g), moved=origin; Vec2d d(0.02,-0.03); moved.boxplus(d);
        require((moved.boxminus(origin)-d).norm()<1e-10,"S2 round trip");
        Eigen::Matrix<double,3,2> numerical;
        for (int i=0;i<2;++i) {
            Vec2d delta=d; delta[i]+=1e-6; S2 plus=origin;plus.boxplus(delta);
            delta=d;delta[i]-=1e-6;S2 minus=origin;minus.boxplus(delta);
            numerical.col(i)=(plus.vec_-minus.vec_)/2e-6;
        }
        require((numerical-origin.S2_Mx(d)).norm()<1e-6,"S2 finite difference Jacobian");
    }
    Vec3d d(.01,-.02,.03);
    for(double scale : {0.0,0.5,-0.5,1e-9})
        require((math::exp(d,scale).matrix()-SO3::exp(2*scale*d).matrix()).norm()<1e-12,"quaternion exp scale");
    PointVector plane;
    plane.push_back(point(-1,-1,0,0));plane.push_back(point(1,-1,0,0));
    plane.push_back(point(1,1,0,0));plane.push_back(point(-1,1,0,0));plane.push_back(point(0,0,0,0));
    Vec4f coef; require(esti_plane(coef,plane,.01),"plane through origin");
    for(auto& p:plane) p.y=0;
    require(!esti_plane(coef,plane,.01),"collinear neighbors rejected");
}
void imuChecks() {
    ImuProcess process; ESKF filter; CloudPtr out(new PointCloudType());
    const Vec3d a(0.4,-0.2,9.79); const double epoch=1700000000.;
    for(int frame=0;frame<106;++frame) {
        MeasureGroup m; m.lidar_begin_time_=epoch+frame*.1; m.lidar_end_time_=m.lidar_begin_time_+.1;
        m.scan_.reset(new PointCloudType());m.scan_->push_back(point(3,1,2,0));m.scan_->push_back(point(3,1,2,100));
        for(int k=0;k<20;++k) m.imu_.push_back(imu(m.lidar_begin_time_+k*.005,a));
        process.Process(m,filter,out);
        if(process.IsIMUInited() && !out->empty()) {
            require(std::abs(filter.GetX().timestamp_-m.lidar_end_time_)<1e-6,"IMU scan-end timestamp");
            require(filter.GetX().pos_.norm()<1e-8 && filter.GetX().vel_.norm()<1e-8,"stationary IMU drift");
            require((out->front().getVector3fMap()-Vec3f(3,1,2)).norm()<1e-5,"stationary deskew");
        }
    }
    require(process.IsIMUInited(),"IMU initialization completed");
    ImuProcess moving; ESKF moving_filter;
    MeasureGroup moving_measure; moving_measure.imu_.push_back(imu(1.0,a,Vec3d(0,0,0.5)));
    moving.Process(moving_measure,moving_filter,out);
    require(!moving.IsIMUInited(),"rotating startup must not initialize gyro bias");
    MeasureGroup empty; process.Process(empty,filter,out);require(out->empty(),"no stale cloud on empty IMU");
    // Constant yaw rate: early and late returns of one world landmark must coincide after deskew.
    auto state=filter.GetX();state.rot_=SO3();state.grav_=S2(Vec3d(0,0,-G_m_s2));
    state.bg_=Vec3d(0,0,-1); state.ba_=Vec3d::Zero(); filter.ChangeX(state);
    MeasureGroup m; m.lidar_begin_time_=state.timestamp_;m.lidar_end_time_=state.timestamp_+.1;
    m.scan_.reset(new PointCloudType());
    for(int k=0;k<=10;++k) {double t=.01*k; Vec3d p=SO3::exp(Vec3d(0,0,-t))*Vec3d(3,1,0);m.scan_->push_back(point(p.x(),p.y(),p.z(),t*1000));}
    for(int k=1;k<=20;++k)m.imu_.push_back(imu(state.timestamp_+k*.005,Vec3d(0,0,a.norm())));
    process.Process(m,filter,out);
    const Vec3f expected=(SO3::exp(Vec3d(0,0,-.1))*Vec3d(3,1,0)).cast<float>();
    require(!out->empty(),"rotating scan output");
    for(const auto& p:*out)require((p.getVector3fMap()-expected).norm()<.001,"rotation deskew angle (not double)");
}
void filterChecks() {
    NavState x;x.grav_=S2(Vec3d(0,0,-G_m_s2));x.pos_=Vec3d(0,0,1);
    ESKF f(x); ESKF::Options options;options.epsi_.setConstant(1e-6);
    bool valid=true;
    options.lidar_obs_func_=[&](NavState& state,ESKF::CustomObservationModel& o) {
        o.valid_=valid;o.h_x_=Eigen::MatrixXd::Zero(30,12);o.h_x_.col(2).setOnes();
        o.residual_=Eigen::VectorXd::Constant(30,-state.pos_.z());o.lidar_residual_mean_=state.pos_.z()*state.pos_.z();
    };f.Init(options);
    require(f.Update(ESKF::ObsType::LIDAR,.001),"valid lidar update");
    require(std::abs(f.GetX().pos_.z())<.001,"height correction");
    require((f.GetP()-f.GetP().transpose()).norm()<1e-10,"symmetric covariance");
    require(Eigen::SelfAdjointEigenSolver<ESKF::CovType>(f.GetP()).eigenvalues().minCoeff()>-1e-10,"PSD covariance");
    auto before=f.GetX();auto covariance=f.GetP();valid=false;
    require(!f.Update(ESKF::ObsType::LIDAR,.001),"invalid update rejected");
    require(NavState(f.GetX()).boxminus(before).norm()<1e-12 && (f.GetP()-covariance).norm()<1e-12,"rejection restores prior");
}
void coreChecks(const char* config) {
    LaserOdometryCore core;require(core.Init(config),"core init");
    int outputs=0;
    for(int frame=0;frame<40;++frame) {
        const double begin=100.+frame*.1;
        for(int k=0;k<20;++k)core.ProcessIMU(imu(begin+k*.005,Vec3d(0,0,G_m_s2)));
        CloudPtr scan(new PointCloudType());
        for(int i=-20;i<=20;++i)for(int j=-20;j<=20;++j) {
            float a=i*.2f,b=j*.2f;
            scan->push_back(point(a,b,-1.5,0));scan->push_back(point(5,a,b,0));scan->push_back(point(a,5,b,0));
        }
        for(size_t i=0;i<scan->size();++i)scan->points[i].time=95.0*i/(scan->size()-1);
        core.ProcessPointCloud(scan,begin);
        if(core.Run()) {
            ++outputs;require(core.GetState().pos_.norm()<.005,"stationary scan matching drift");
            require(std::abs(core.GetState().timestamp_-(begin+.095))<1e-8,"original scan end");
        }
    }
    require(outputs>25,"sufficient accepted stationary scans");
    // Large scan gap with continuous IMU: must resume matching, not latch into skip forever.
    for(int k=0;k<140;++k) core.ProcessIMU(imu(104.+k*.005,Vec3d(0,0,G_m_s2)));
    CloudPtr recovery(new PointCloudType());
    for(int i=-20;i<=20;++i)for(int j=-20;j<=20;++j) {
        recovery->push_back(point(i*.2f,j*.2f,-1.5,95));
        recovery->push_back(point(5,i*.2f,j*.2f,0));
    }
    recovery->back().time = 95.0;
    core.ProcessPointCloud(recovery,104.6);
    require(core.Run(),"resume after >0.5 second LiDAR gap");
    const double accepted_stamp=core.GetState().timestamp_;
    for(int k=0;k<20;++k)core.ProcessIMU(imu(104.7+k*.005,Vec3d(0,0,G_m_s2)));
    CloudPtr unmatched(new PointCloudType());
    for(int i=0;i<100;++i)unmatched->push_back(point(100+i,100,100,i*.95));
    core.ProcessPointCloud(unmatched,104.7);
    require(!core.Run(),"unmatched scan rejected instead of using stale neighbors");
    require(core.GetState().timestamp_==accepted_stamp,"failed match not published as accepted pose");
    // Timestamp rewind starts a new epoch and waits for initialization.
    core.ProcessIMU(imu(1.,Vec3d(0,0,G_m_s2)));
    require(core.GetScanDownWorld()->empty(),"time regression clears local scan/map epoch");
    std::cout << "stationary matched frames=" << outputs << " position=" << core.GetState().pos_.transpose() << '\n';
}
void pointInputChecks(const char* config) {
    PointCloudPreprocess preprocess;
    preprocess.Set(LidarType::OUST64, 1.2, 1);
    auto raw = boost::make_shared<pcl::PointCloud<OusterPoint>>();
    for (int i=0;i<3;++i) {
        OusterPoint p{}; p.x=3; p.y=i; p.z=-1; p.intensity=10; p.t=i*50000000;
        raw->push_back(p);
    }
    pcl::PCLPointCloud2 message;
    pcl::toPCLPointCloud2(*raw,message);
    auto decoded=boost::make_shared<pcl::PointCloud<OusterPoint>>();
    pcl::fromPCLPointCloud2(message,*decoded);
    CloudPtr converted(new PointCloudType());preprocess.Process(decoded,converted);
    require(converted->size()==3 && converted->back().time==100.0,"original Ouster t nanoseconds to ms");
    pcl::toPCLPointCloud2(*converted,message);
    bool timestamp_field=false;
    for(const auto& field:message.fields) timestamp_field |= field.name=="timestamp";
    require(timestamp_field,"original internal timestamp field registration");

    // Both all-zero-from-start and zero-after-valid-time must follow original fallback.
    for(bool history : {false,true}) {
        LaserOdometryCore core;require(core.Init(config),"zero-time core init");
        int accepted=0;
        for(int frame=0;frame<24;++frame) {
            double begin=200.+frame*.1;
            for(int k=0;k<=20;++k)core.ProcessIMU(imu(begin+k*.005,Vec3d(0,0,G_m_s2)));
            auto cloud=boost::make_shared<pcl::PointCloud<OusterPoint>>();
            for(int i=-15;i<=15;++i)for(int j=-15;j<=15;++j) {
                OusterPoint p{};p.x=i*.2f;p.y=j*.2f;p.z=-1.5f;p.intensity=1;
                p.t=(history && frame<12) ? 95000000u : 0u;
                cloud->push_back(p);
            }
            core.ProcessPointCloud(cloud,begin);
            if(core.Run()) {
                ++accepted;
                double expected=begin+(history ? .095 : 0.0);
                require(std::abs(core.GetState().timestamp_-expected)<1e-8,"original zero-time scan duration fallback");
                require(core.GetState().pos_.norm()<.005,"zero-time stationary height");
            }
        }
        require(accepted>12,"zero point times must not reject scans");
    }
}
// ============================================================================
// 传感器数据健康 / 轮速插值 / 静止滞回 —— 本次重构的定向回归
//
// 其中 P0-1 / P0-2 两条直接对应实测暴露的缺陷：
//   P0-1 轮速时间戳与 LiDAR 不同源（旧实现回退墙钟）→ 非法时间戳必须被丢弃
//   P0-2 [t_beg,t_end] 窗口过滤丢掉插值锚点       → 锚点必须是"最后一个 <= t_end 的样本"
// ============================================================================
void sensorHealthChecks(const char* config) {
    SensorHealthMonitor::Options opt;

    // ---- 1. 有无 + 新鲜度：三个条件缺一不可 ----
    {
        SensorHealthMonitor mon; mon.Configure(true, true, opt);
        auto s = mon.Evaluate(100.0);
        require(!s.wheel.usable(), "never received must be unusable");
        require(!s.gnss.usable(), "gnss never received must be unusable");
    }
    {
        SensorHealthMonitor mon; mon.Configure(true, true, opt);
        mon.OnWheel(99.95);
        auto s = mon.Evaluate(100.0);
        require(s.wheel.usable(), "fresh sample must be usable");
        require(std::abs(s.wheel.age - 0.05) < 1e-9, "age = t_ref - last_stamp");
    }
    {
        SensorHealthMonitor mon; mon.Configure(true, true, opt);
        mon.OnWheel(99.0);                       // age = 1.0s > 0.15s
        require(!mon.Evaluate(100.0).wheel.usable(), "stale sample must be unusable");
    }
    {
        SensorHealthMonitor mon; mon.Configure(false, true, opt);   // 配置关
        mon.OnWheel(99.95);
        require(!mon.Evaluate(100.0).wheel.usable(), "disabled config must be unusable even if fresh");
    }
    {
        // 实时流水线：last_stamp 天然领先 t_ref 一个流水线延迟（传输+组包+调度+处理）。
        // 这是"数据在流动"的正常表现，必须判为可用——否则实车上 GNSS/轮速会被误杀。
        SensorHealthMonitor mon; mon.Configure(true, true, opt);
        mon.OnWheel(100.114);                    // 超前 114ms，与实测日志 age=-0.114 同量级
        auto s = mon.Evaluate(100.0);
        require(s.wheel.usable(), "small pipeline lead must stay usable");
        require(std::abs(s.wheel.age + 0.114) < 1e-9, "negative age means data leads the frame");
    }
    {
        // 超前离谱 → 时基不同源（如 GNSS 接收机时钟 vs LiDAR GPS 时钟）
        SensorHealthMonitor mon; mon.Configure(true, true, opt);
        mon.OnWheel(200.0);                      // 超前 100s >> max_clock_skew_sec
        require(!mon.Evaluate(100.0).wheel.usable(), "absurd clock skew must be unusable");
    }
    {
        SensorHealthMonitor mon; mon.Configure(true, true, opt);
        mon.OnGnss(99.5, 33, 12);
        auto s = mon.Evaluate(100.0);
        require(s.gnss.usable(), "fresh gnss must be usable");
        require(s.gnss.quality_status == 33 && s.gnss.quality_nsv == 12, "gnss quality is carried through");
    }

    // ---- 2. 静止滞回 ----
    {
        StationaryDetector det;
        StationaryDetector::Options sopt;
        sopt.min_speed_mps = 0.2; sopt.resume_speed_mps = 0.4;
        sopt.confirm_frames = 3;  sopt.resume_hold_frames = 2;
        det.Configure(sopt);

        require(!det.Update(0.1) && !det.confirmed(), "hold 1/3 not yet confirmed");
        det.Update(0.1);
        require(det.Update(0.1) && det.confirmed(), "hold 3/3 confirms stationary");
        require(det.Update(0.3) && det.confirmed(), "between thresholds keeps state (hysteresis)");
        require(!det.Update(0.5) && !det.confirmed(), "above resume speed clears stationary");
        require(det.resume_hold() == 2, "resume hold inherited from confirmed state");
        det.TickResumeHold();
        require(det.resume_hold() == 1, "resume hold counts down");
        det.TickResumeHold();
        require(det.resume_hold() == 0, "resume hold reaches zero");
    }

    // ---- 3. 轮速时间戳合法性（P0-1）----
    {
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        double v = -1.0;
        core.FeedWheelSpeed(0.0, 30.0, 1);       // stamp==0 视为无效（与 ROS 层判定一致）
        core.FeedWheelSpeed(-5.0, 30.0, 1);
        require(!core.ConsumeWheelSpeedAt(100.0, &v), "invalid stamps must not be registered");
    }

    // ---- 4. 轮速取值：锚点 = 最后一个 <= t_end（P0-2 定向回归）----
    {
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        // 两枚样本都落在 t_beg=10.08 之前；旧实现会因"帧区间内无样本"整帧放弃
        core.FeedWheelSpeed(10.00, 18.0, 1);     // 5.0 m/s
        core.FeedWheelSpeed(10.05, 25.2, 1);     // 7.0 m/s
        double v = -1.0;
        require(core.ConsumeWheelSpeedAt(10.10, &v), "anchor before t_beg must still yield a value");
        require(std::abs(v - 7.0) < 1e-9, "anchor is the latest sample <= t_end");
    }
    {
        // 年龄门：锚点太老则不约束
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        core.FeedWheelSpeed(10.00, 18.0, 1);
        double v = -1.0;
        require(!core.ConsumeWheelSpeedAt(11.00, &v), "stale wheel must be rejected by age gate");
    }
    {
        // 全部在未来：等下一帧
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        core.FeedWheelSpeed(10.50, 18.0, 1);
        double v = -1.0;
        require(!core.ConsumeWheelSpeedAt(10.00, &v), "all-future wheel must be rejected");
    }

    // ---- 5. 插值正确性 + 开关 ----
    {
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        core.SetWheelInterpolation(true);
        core.FeedWheelSpeed(10.00, 0.0, 1);      // 0.0 m/s
        core.FeedWheelSpeed(10.10, 36.0, 1);     // 10.0 m/s
        double v = -1.0;
        require(core.ConsumeWheelSpeedAt(10.05, &v), "interpolation window");
        require(std::abs(v - 5.0) < 1e-9, "linear interpolation at midpoint");
    }
    {
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        core.SetWheelInterpolation(false);       // 退化为"取最新 <= t_end"
        core.FeedWheelSpeed(10.00, 0.0, 1);      // 0.0 m/s
        core.FeedWheelSpeed(10.10, 36.0, 1);     // 10.0 m/s
        double v = -1.0;
        require(core.ConsumeWheelSpeedAt(10.05, &v), "no-interpolation path");
        require(std::abs(v - 0.0) < 1e-9, "no-interpolation returns the latest sample <= t_end");
    }
    {
        // t_end 恰为后一个样本时刻：alpha=1，取到后样本（clamp 上边界）
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        core.SetWheelInterpolation(true);
        core.FeedWheelSpeed(10.00, 0.0, 1);
        core.FeedWheelSpeed(10.10, 36.0, 1);
        double v = -1.0;
        require(core.ConsumeWheelSpeedAt(10.10, &v), "t_end at later sample");
        require(std::abs(v - 10.0) < 1e-9, "alpha clamps to 1 at the later sample");
    }
    {
        // gear==2（倒挡）速度取负
        LaserOdometryCore core; require(core.Init(config), "core init for wheel");
        core.FeedWheelSpeed(10.00, 18.0, 2);
        double v = -1.0;
        require(core.ConsumeWheelSpeedAt(10.02, &v), "reverse sample");
        require(std::abs(v + 5.0) < 1e-9, "reverse gear negates speed");
    }
}

// 每段独立执行：一段失败不遮蔽其余段的结果（便于区分"新改动的回归"与"既有失败"）。
// 任一段失败则进程返回 1。
int main(int argc,char**argv) {
    if (argc != 2) { std::cerr << "FAIL: supply config YAML\n"; return 1; }

    bool ok = true;
    auto run = [&ok](const char* name, auto&& fn) {
        try { fn(); std::cout << "  [PASS] " << name << '\n'; }
        catch (const std::exception& e) { std::cout << "  [FAIL] " << name << ": " << e.what() << '\n'; ok = false; }
    };

    run("math", [] { mathChecks(); });
    run("imu", [] { imuChecks(); });
    run("filter", [] { filterChecks(); });
    run("core", [&] { coreChecks(argv[1]); });
    run("pointInput", [&] { pointInputChecks(argv[1]); });
    run("sensorHealth", [&] { sensorHealthChecks(argv[1]); });

    if (!ok) {
        std::cerr << "FAIL: one or more regression sections failed (see above)\n";
        return 1;
    }
    std::cout << "PASS: all regression sections\n";
    return 0;
}
