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
int main(int argc,char**argv) {
    try { require(argc==2,"supply config YAML"); mathChecks();imuChecks();filterChecks();coreChecks(argv[1]);pointInputChecks(argv[1]); }
    catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
    std::cout<<"PASS: math, plane, IMU, deskew, ESKF, and stationary point-cloud regressions\n";
}
