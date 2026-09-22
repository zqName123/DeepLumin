#include <iostream>
#include <stdexcept>
#include <fstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include "lio_odometry/core/laser_odometry_core.h"
#include "lio_odometry/core/voxelmap/point_covariance.h"
using namespace slam_odometry;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
pointWithVar point(double x,double y,double z) {
    pointWithVar p;p.point_b=p.point_i=p.point_w=Vec3d(x,y,z);
    p.var=p.body_var=Mat3d::Identity()*.0004;return p;
}
void covariance() {
    NavState s;s.rot_=SO3::exp(Vec3d(.1,.2,-.3));s.offset_R_lidar_=SO3::exp(Vec3d(.3,-.5,.8));
    s.offset_t_lidar_=Vec3d(.2,.3,.4);s.grav_=S2(Vec3d(0,0,-9.809));
    Vec3d p(4,2,1); Mat3d sensor=Vec3d(.001,.002,.009).asDiagonal();
    ESKF::CovType P=ESKF::CovType::Zero();P.diagonal().head<12>().setLinSpaced(.001,.012);
    P(0,4)=P(4,0)=.0002;
    Eigen::Matrix<double,3,23> J=Eigen::Matrix<double,3,23>::Zero();
    for(int i=0;i<12;++i) {
        NavState::VectState d=NavState::VectState::Zero();d[i]=1e-6;
        NavState plus=s.boxplus(d), minus=s.boxplus(-d);
        J.col(i)=(plus.rot_*(plus.offset_R_lidar_*p+plus.offset_t_lidar_)+plus.pos_-
                  (minus.rot_*(minus.offset_R_lidar_*p+minus.offset_t_lidar_)+minus.pos_))/2e-6;
    }
    Mat3d R=(s.rot_*s.offset_R_lidar_).matrix();
    Mat3d expected=R*sensor*R.transpose()+J*P*J.transpose();
    check((expected-PointCovarianceWorld(s,P,p,sensor,true)).norm()<1e-8,"covariance: index/extrinsic/cross terms");
    for(Vec3d v:{Vec3d(10,0,0),Vec3d(0,0,10),Vec3d::Zero().eval()}) {
        Mat3d result;result.setConstant(std::numeric_limits<double>::quiet_NaN());
        VoxelMapManagerFASTLIVO::CalcBodyCov(v,.02,.05,result);
        check(result.allFinite(),"body covariance z=0");
        check(Eigen::SelfAdjointEigenSolver<Mat3d>(result).eigenvalues().minCoeff()>-1e-12,"body covariance PSD");
    }
    S2 g(Vec3d(0,0,-9.809)), moved=g;Vec2d d(.02,-.03);moved.boxplus(d);
    check((moved.boxminus(g)-d).norm()<1e-10,"S2 half angle");
}
void filter() {
    for(bool aa:{false,true}) {
        ESKF f; ESKF::Options o;o.use_aa_=aa;o.epsi_.setConstant(1e-9);o.max_iterations_=4;
        o.lidar_converged_times_required_=2;
        bool valid=true;
        o.lidar_obs_func_=[&](NavState& s, ESKF::CustomObservationModel& m) {
            m.valid_=valid;m.h_x_=Eigen::MatrixXd::Zero(2,12);m.h_x_.col(2).setOnes();
            m.residual_.resize(2);m.residual_<<1-s.pos_.z(),-1-s.pos_.z();
            m.variance_.resize(2);m.variance_<<.01,1.;m.lidar_residual_mean_=0;
        };
        f.Init(o);check(f.Update(ESKF::ObsType::LIDAR,.001),"weighted update");
        check(std::abs(f.GetX().pos_.z()-99./102.)<1e-6,"unequal measurement variances influence estimate");
        check(f.GetIterations()>=3,"consecutive convergence requirement");
        auto before=f.GetX();auto P=f.GetP();valid=false;
        check(!f.Update(ESKF::ObsType::LIDAR,.001),"reject invalid update");
        check(NavState(f.GetX()).boxminus(before).norm()<1e-12&&(f.GetP()-P).norm()<1e-12,"rejection rollback");
    }
}
void planes() {
    VoxelMapConfig cfg;cfg.use_ransac_=true;cfg.valid_check_resolution_=40;cfg.ransac_isplane_p_threshold_=.8;
    VoxelOctoTreeFASTLIVO tree(2,0,5,1000,.01);tree.config_ptr_=&cfg;tree.quater_length_=2.5;
    tree.use_ransac_=true;tree.layer_init_num_={5,5,5};
    // One large connected patch, one isolated coplanar point. Valid check removes the latter.
    for(int i=0;i<10;++i)for(int j=0;j<10;++j)tree.new_points_.push_back(point(1+i*.04,1+j*.04,2));
    tree.new_points_.push_back(point(8,8,2));tree.init_plane_ransac();
    check(tree.plane_ptr_->is_plane_,"ransac plane retained");
    check(tree.plane_points_.size()==100,"connected component pruned");
    check((tree.plane_ptr_->center_-Vec3d(1.18,1.18,2)).norm()<1e-8,"centroid after pruning");
    check(tree.plane_ptr_->plane_var_.allFinite(),"finite plane covariance");
    tree.reinit_size_vec_={10,20};tree.all_points_num_root_=100;tree.init_octo_=true;
    tree.new_points_.push_back(point(1,1,2));tree.update_octo_tree();
    check(tree.stop_reinit_,"reinit crossed last threshold");
    tree.update_octo_tree(); // no out-of-bounds on subsequent updates
}
void neighbors() {
    for(int radius:{0,1}) {
        VoxelMapConfig cfg;cfg.max_voxel_size_=3;cfg.max_layer_=0;cfg.use_ransac_=false;
        cfg.neighbor_search_radius_=radius;cfg.reinit_size_vec_.clear();
        VoxelMapManagerFASTLIVO map;map.Init(cfg);
        std::vector<pointWithVar> pts;
        for(int i=0;i<8;++i)for(int j=0;j<8;++j)pts.push_back(point(2.5+i*.06,1+j*.08,2));
        map.UpdateMap(pts);std::vector<PointToPlane> matches;
        map.BuildResidualListOMP({point(3.01,1.25,2)},matches);
        check(matches.size()==static_cast<size_t>(radius==1),"neighbor search when current root absent");
        pts={point(-3,1,2)};map.UpdateMap(pts);
        check(map.GetMap().count(VOXEL_LOCATION(-1,0,0))==1,"negative voxel boundary uses floor");
    }
}
void config(const char* path) {
    char tmp[]="/tmp/voxel-config-XXXXXX";int fd=mkstemp(tmp);check(fd>=0,"temp config");close(fd);
    for(bool aa:{false,true}) {
        auto yaml=YAML::LoadFile(path);yaml["fasterlio"]["use_aa"]=aa;
        { std::ofstream out(tmp);out<<yaml; }
        LaserOdometryCore core;check(core.Init(tmp),"core config init");
        check(core.GetESKF().GetUseAA()==aa,"AA comes from YAML");
    }
    unlink(tmp);
}
void pipeline(const char* path, bool mixed = false) {
    LaserOdometryCore core;check(core.Init(path),"pipeline init");int accepted=0;double max_drift=0;
    for(int f=0;f<25;++f) {
        const double begin=100.+.1*f;
        for(int k=0;k<=20;++k) {
            auto imu=std::make_shared<IMU>();imu->timestamp=begin+.005*k;
            imu->linear_acceleration=Vec3d(0,0,9.809);core.ProcessIMU(imu);
        }
        CloudPtr cloud(new PointCloudType());
        for(int i=-20;i<=20;++i)for(int j=-20;j<=20;++j) {
            PointType p;p.x=i*(mixed?.2f:.1f);p.y=j*(mixed?.2f:.1f);p.z=-1.5f;p.intensity=1;p.time=0;cloud->push_back(p);
            p.x=mixed?5:8;p.y=i*(mixed?.2f:.1f);p.z=(mixed?0:3)+j*(mixed?.2f:.1f);cloud->push_back(p);
            // Third plane constrains motion along the intersection of floor and first wall.
            p.x=i*.1f;p.y=-8;p.z=3+j*.1f;cloud->push_back(p);
        }
        cloud->back().time=95.;core.ProcessPointCloud(cloud,begin);
        if(core.Run()) {
            ++accepted;
            max_drift=std::max(max_drift,core.GetState().pos_.norm());
            check(core.GetState().pos_.allFinite(),"finite pipeline pose");
            if(!mixed)check(core.GetState().pos_.norm()<.01,"stationary separated-plane drift");
            const auto& P=core.GetESKF().GetP();
            check(P.allFinite()&&(P-P.transpose()).norm()<1e-8,"pipeline covariance");
        }
    }
    check(accepted>15,"pipeline accepted frames");
    std::cout<<std::setprecision(9)<<"stationary mixed="<<mixed<<" accepted="<<accepted<<" max_drift_m="<<max_drift<<" position="<<core.GetState().pos_.transpose()<<'\n';
}
int main(int argc,char**argv) {
    try {check(argc==2,"config path");covariance();filter();planes();neighbors();config(argv[1]);pipeline(argv[1]);pipeline(argv[1],true);}
    catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
    std::cout<<"PASS: covariance, weighted ESKF/AA, plane pruning, reinit, neighbors, YAML\n";
}
