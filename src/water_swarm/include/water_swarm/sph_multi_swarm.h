#ifndef SPH_MULTI_SWARM_H
#define SPH_MULTI_SWARM_H

#define PI 3.14159265f

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/topic_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <regex>
#include <map>
#include <random>
#include <algorithm>
#include <thread>
#include <math.h>
#include <unordered_map>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "common_msgs/Position.h"
#include "common_msgs/Velocity.h"
#include "common_msgs/Acceleration.h"
#include "common_msgs/Force.h"
#include "common_msgs/Odom.h"
#include "common_msgs/OdomWithNeighbors.h"
#include "common_msgs/OdomBroadcast.h"
#include "common_msgs/Particle.h"
#include "common_msgs/Swarm_particles.h"
#include "common_msgs/PositionCommand.h"
#include "common_msgs/BsplineTraj.h"
#include "common_msgs/Swarm_traj.h"

using namespace std;

// 全局变量定义
ros::Timer timer;
ros::Publisher particles_publisher;
ros::Publisher virtual_particles_publisher;
ros::Publisher swarm_pub;
ros::Subscriber swarm_traj_sub;
ros::Subscriber target_sub;
ros::Subscriber force_sub;
ros::Subscriber collision_matrix_sub;
ros::Publisher pos_pub;
ros::Publisher vel_pub;
ros::Publisher initial_pospub;
std::vector<ros::Publisher> odom_publishers;

ros::Time current_time;
ros::Time last_time;
ros::Time last_print_time;
bool receive_target = false;

int particleCount;
int groupCount;  // 新增：组数量
bool use_random_sort;  // 是否使用随机排序
double particleInterval;
double particleVisScale;
float mass;
float restDensity;
float h;
float g;
double updateInterval;
double threshold_dist;
double k_den;
double k_rep;
double k_fri;
double k_p;
double k_d;
double k_ff;
double v_max;
double a_max;
double r_1;
double r_2;
bool state_enabled;
bool vis_role;
double init_bias_x;
double init_bias_y;
double d_AA;
double d_AB;
double k_group_sep;
bool avoid_enabled_ = true;
double eta_ = 0.2;
double r_aoi_scale_ = 1.2;
double c_gamma_pen_ = 4.0;

// 回调函数声明
void odomBroadcastCallback(const common_msgs::OdomBroadcast::ConstPtr& msg);
void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);
void swarmTrajCallback(const common_msgs::Swarm_traj::ConstPtr& msg);
void targetCallback(const common_msgs::Swarm_particles::ConstPtr& msg);
void forceCallback(const common_msgs::Swarm_particles::ConstPtr& msg);
void collisionMatrixCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){};
void goalCallback(const common_msgs::Swarm_particles::ConstPtr& msg){};
void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


struct SPHSettings
{   
    // 添加的默认构造函数
    SPHSettings()
    : mass(1.0f),            // 默认质量
      restDensity(1.0f),     // 默认静止密度，水的密度大约是1000kg/m^3
      h(1.0f),               // 影响半径，决定粒子影响的范围
      g(-9.81f)              // 重力加速度，向下是负值
    {
        h2 = h * h;
        poly = 10 * 54.97 / ( 7 * PI * h2);
        selfDens = mass * poly ;
    };

    SPHSettings(
        float mass, float restDensity, float h, float g)
    : mass(mass)
    , restDensity(restDensity)
    , h(h)
    , g(g)
    {
        h2 = h * h;
        poly = 10 * 54.97 / ( 7 * PI * h2);
        // poly = 1 / (  PI * h2* h);
        selfDens = mass * poly ;
    }

    float poly, polyGrad, mass, h2, selfDens, restDensity,  h, g;
};

enum ParticleState {
    NULL_STATE,  // "NULL" 状态
    TRAJ,        // "TRAJ" 状态
    NEED_TRAJ,   // 需要重规划
    ATTRACT,     // "吸引" 状态，对应英文 "attract"
    REPEL,        // "排斥" 状态，对应英文 "repel"
    NEAR_TARGET  // 靠近目标状态
};

enum ParticleRole {
    LEADER,
    FOLLOWER,
    FREE,
    VIRTUAL
};

struct Particle
{
    common_msgs::Position       position;
    common_msgs::Velocity       velocity;
    common_msgs::Acceleration   acceleration;
    common_msgs::Force          force;
    float                       density;
    float                       pressure;
    uint16_t                    hash;
    common_msgs::Force          u_den, u_rep, u_fri;
    std_msgs::String            name;
    int                         index;
    int                         group_id; 
    ParticleState               state = NULL_STATE;
    ParticleRole                role  = FREE;
    common_msgs::Force          u_diff;
    
    inline bool isVirtual() const { return role == VIRTUAL; }
};

class SPHSystem
{
public:
    SPHSettings     settings;
    size_t          sph_particleCubeWidth;
    size_t          sph_particleCount;
    
    // 避碰参数
    double eta_ = 0.2;
    double r_aoi_scale_ = 1.2;
    double c_gamma_pen_ = 4.0;

    bool started;
    bool runOnGPU;
    bool isInitialReceived = false;  // 用于检查是否已经接收到第一个消息
    ros::Time start_time;  // 记录开始时间

public:
	SPHSystem(
        size_t numParticles, const SPHSettings &settings,
        const bool &runOnGPU);
    SPHSystem();
	~SPHSystem();

    std::vector<Particle>       particles;
    std::vector<Particle>       virtual_particles;
    std::map<const Particle*, std::vector<std::pair<const Particle*, float>>> particleNeighborsTable;
    std::map<const Particle*, double> nearestNeighborDistanceMap;
    std::map<const Particle*, std::vector<const Particle*>> collisionNeighborsTable;
    Eigen::MatrixXi collision_matrix_;

    //initializes the particles that will be used
	void initParticles();
    void findNeighbors();
    void updateParticleStates();

	void update(float deltaTime);
	void reset();
	void start();
    void stop();

    /// Update attrs of particles in place.
    void updateParticles(
        std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime, const bool onGPU);

    void updateParticlesGPU(
        std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime){};

    void updateParticlesCPU(
        std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
        float deltaTime);
        
    void processTraj(const common_msgs::Swarm_traj& swarmtraj);
    bool isNearVirtualParticle(double x, double y, double z);
    void generateVirtualParticles(const double l, const int particlesPerSide, const common_msgs::Position& apex);
    void parallelDensityAndPressures();
    void parallelForces();
    void parallelViscosity();
    void parallelUpdateParticleTraj();
    void parallelUpdateParticlePositions(const float deltaTime);
    void computeGroupDifferentialForces();
    void pubroscmd();
    
    // 避碰相关函数
    bool buildBarrierConstraint2D(
        const Particle& pi, const Particle& pj,
        double R, double gamma,
        Eigen::Vector2d& a_ij, double& b_ij, double& h_ij);
    
    Eigen::Vector2d projectOntoHalfspace2D(
        const Eigen::Vector2d& v0,
        const Eigen::Vector2d& a, double b);
    
    Eigen::Vector2d solveMinChangeVelocity2D(
        const Eigen::Vector2d& v0,
        const std::vector<Eigen::Vector2d>& A,
        const std::vector<double>& B,
        double vmax);
    
    void applyVelocityBarrierOnce(
        Particle& p, float dt,
        double R, double gamma, double r_aoi, double vmax);
    
    void avoidCollisionVelocityStep(float dt);

    inline std::string stateToString(ParticleState state) {
        switch (state) {
            case NULL_STATE: return "SPH";
            case TRAJ:       return "TRAJ";
            case NEED_TRAJ:  return "NEED";
            case ATTRACT:    return "ATTRACT";
            case REPEL:      return "REPEL";
            case NEAR_TARGET:return "NEAR";
            default:        return "UNKNOWN"; // 处理未知状态
        }
    };

    inline std::string roleToString(ParticleRole role) {
        switch (role) {
            case FREE: return "FREE";
            case LEADER:       return "LEADER";
            case FOLLOWER:  return "FOLLOWER";
            default:        return "UNKNOWN"; // 处理未知状态
        }
    };

    inline double clamp(double value, double max_value) 
    {
        if (value > max_value) {
            return max_value;
        } else if (value < -max_value) {
            return -max_value;
        }
        return value;
    }

    inline void updateCollisionMatrix(const Eigen::MatrixXi& matrix)
    {
        collision_matrix_ = matrix;
    }

    inline void setParticleColor(int group_id, std_msgs::ColorRGBA& color) {
        // 使用12种不同的颜色
        switch (group_id % 12) {
            case 0:  // 红色
                color.r = 1.0; color.g = 0.0; color.b = 0.0;
                break;
            case 1:  // 绿色
                color.r = 0.0; color.g = 1.0; color.b = 0.0;
                break;
            case 2:  // 蓝色
                color.r = 0.0; color.g = 0.0; color.b = 1.0;
                break;
            case 3:  // 黄色
                color.r = 1.0; color.g = 1.0; color.b = 0.0;
                break;
            case 4:  // 紫色
                color.r = 1.0; color.g = 0.0; color.b = 1.0;
                break;
            case 5:  // 青色
                color.r = 0.0; color.g = 1.0; color.b = 1.0;
                break;
            case 6:  // 橙色
                color.r = 1.0; color.g = 0.5; color.b = 0.0;
                break;
            case 7:  // 粉色
                color.r = 1.0; color.g = 0.4; color.b = 0.7;
                break;
            case 8:  // 棕色
                color.r = 0.6; color.g = 0.3; color.b = 0.0;
                break;
            case 9:  // 深绿色
                color.r = 0.0; color.g = 0.5; color.b = 0.0;
                break;
            case 10: // 深蓝色
                color.r = 0.0; color.g = 0.0; color.b = 0.5;
                break;
            case 11: // 灰色
                color.r = 0.5; color.g = 0.5; color.b = 0.5;
                break;
        }
        color.a = 1.0;  // 设置透明度
    }

};
#endif // SPH_MULTI_SWARM_H 