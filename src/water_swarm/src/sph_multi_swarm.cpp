#include "sph_multi_swarm.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_particles");
    ros::NodeHandle nh("~");

    nh.param("sph/particleCount", particleCount, -1);
    nh.param("sph/groupCount", groupCount, 1);
    nh.param("sph/particleInterval", particleInterval, 0.1);
    nh.param("sph/particleVisScale", particleVisScale, 0.1);
    nh.param("sph/mass", mass, 1.00f);
    nh.param("sph/restDensity", restDensity, 1.0f);
    nh.param("sph/h", h, 0.15f);
    nh.param("sph/g", g, -9.8f);
    nh.param("sph/updateInterval", updateInterval, 0.01);
    nh.param("sph/threshold_dist", threshold_dist, 0.1);
    nh.param("sph/k_den", k_den, -1.0);
    nh.param("sph/k_rep", k_rep, -1.0);
    nh.param("sph/k_fri", k_fri, -1.0);
    nh.param("sph/k_p", k_p, -1.0);  
    nh.param("sph/k_d", k_d, 0.0);  
    nh.param("sph/k_ff", k_ff, 0.0);
    nh.param("sph/v_max", v_max, -1.0);
    nh.param("sph/a_max", a_max, -1.0);
    nh.param("sph/r_1",   r_1, -1.0);
    nh.param("sph/r_2",   r_2, -1.0);
    nh.param("sph/state_enabled", state_enabled, false); 
    nh.param("sph/vis_role", vis_role, false);  
    nh.param("sph/init_bias_x", init_bias_x, -1.0);  //
    nh.param("sph/init_bias_y", init_bias_y, -1.0);  //
    nh.param("sph/d_AA", d_AA, 1.0);
    nh.param("sph/d_AB", d_AB, 3.0);
    nh.param("sph/k_group_sep", k_group_sep, 1.0);
    nh.param("sph/use_random_sort", use_random_sort, false);
    
    // 避碰相关参数
    nh.param("avoid/enabled", avoid_enabled_, true);
    nh.param("avoid/eta", eta_, 0.2);
    nh.param("avoid/r_aoi_scale", r_aoi_scale_, 1.2);
    nh.param("avoid/c_gamma_pen", c_gamma_pen_, 4.0);
    timer                 = nh.createTimer(ros::Duration(updateInterval),   timerCallback);
    particles_publisher   = nh.advertise<visualization_msgs::MarkerArray>("particles_vis", 10);
    swarm_pub             = nh.advertise<common_msgs::Swarm_particles>("/swarm_particles", 10);
    swarm_traj_sub        = nh.subscribe("/swarm_traj", 1000, swarmTrajCallback);
    target_sub            = nh.subscribe("/particle_target", 10,targetCallback);
    force_sub             = nh.subscribe("/particle_force", 10,forceCallback);
    collision_matrix_sub  = nh.subscribe("/check_collision_matrix", 10,collisionMatrixCallback);
    ros::Subscriber simple_goal_sub = nh.subscribe("/move_base_simple/goal", 10, simpleGoalCallback);
    pos_pub               = nh.advertise<geometry_msgs::PoseStamped>("trajectory_position", 10);
    vel_pub               = nh.advertise<geometry_msgs::PoseStamped>("trajectory_velocity", 10);
    initial_pospub        = nh.advertise<visualization_msgs::MarkerArray>("/initial_pose", 1,true);
    odom_publishers.resize(particleCount * groupCount);
    for (int i = 0; i < particleCount * groupCount; ++i) {
        std::stringstream ss;
        ss << "/particle" << i << "/odom";
        odom_publishers[i] = nh.advertise<nav_msgs::Odometry>(ss.str(), 10);
    }

    //start sph
    SPHSettings sphSettings(mass, restDensity, h, g);
    sph_planner = new SPHSystem(15, sphSettings, false);
    sph_planner->eta_ = eta_;
    sph_planner->r_aoi_scale_ = r_aoi_scale_;
    sph_planner->c_gamma_pen_ = c_gamma_pen_;

    ROS_INFO("sph_planner node has started.");
    ros::spin();
    return 0;
}

void swarmTrajCallback(const common_msgs::Swarm_traj::ConstPtr& msg){}

void timerCallback(const ros::TimerEvent&) {
    
    current_time = ros::Time::now();

    if (!sph_planner->started) 
    {
        if ((current_time - last_print_time).toSec() >= 1.0) {
            ROS_INFO("Do not receive the start command, showing initial particles.");
            last_print_time = current_time;  
        }
        // 即使未启动，也发布初始状态的粒子以便可视化
        sph_planner->pubroscmd();
        return;
    }
    else
    {   
        float deltaTime = last_time.isZero() ? 0.0 : (current_time - last_time).toSec();
        last_time = current_time;
        sph_planner->update(updateInterval);
    }
}

void targetCallback(const common_msgs::Swarm_particles::ConstPtr& msg){}
void forceCallback(const common_msgs::Swarm_particles::ConstPtr& msg){}

void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!sph_planner->started) {
        sph_planner->started = true;
        sph_planner->start_time = ros::Time::now();
        ROS_INFO("Received goal from /move_base/simple_goal, starting SPH system.");
    }
}
void SPHSystem::processTraj(const common_msgs::Swarm_traj& swarmtraj){}

void SPHSystem::initParticles() {
    sph_particleCount = particleCount * groupCount;  // 总粒子数 = 每组粒子数 * 组数
    particles.clear();

    if (!use_random_sort) {
        // 原有的按组排列逻辑
        for (int g = 0; g < groupCount; ++g) {
            int sideLength = static_cast<int>(std::sqrt(particleCount));
            if (sideLength * sideLength < particleCount) {
                ++sideLength;
            }
            double maxRange = sideLength * particleInterval;
            
            // 为每个组初始化粒子
            for (int i = 0; i < particleCount; ++i) {
                int row = i / sideLength;
                int col = i % sideLength;
                
                Particle p;
                p.position.x = col * particleInterval - init_bias_x + g * maxRange;  // 根据组ID偏移x坐标
                p.position.y = row * particleInterval - init_bias_y;
                p.position.z = 1.0;
                p.velocity.x = p.velocity.y = p.velocity.z = 0.0;
                p.acceleration.x = p.acceleration.y = p.acceleration.z = 0.0;
                p.force.x = p.force.y = p.force.z = 0.0;
                p.density = settings.selfDens;
                p.pressure = 0.0;
                p.hash = 0;
                p.index = g * particleCount + i;  // 索引 = 组ID * 每组粒子数 + 组内索引
                p.group_id = g;  // 设置组ID

                std::stringstream ss;
                ss << "Group " << g << " Particle " << i;
                p.name.data = ss.str();

                particles.push_back(p);
            }
        }
    } else {
        // 随机排序逻辑：所有粒子打散在初始空间中
        std::vector<std::pair<int, int>> particle_info;  // (group_id, particle_id)
        
        // 生成所有粒子的组ID和粒子ID信息
        for (int g = 0; g < groupCount; ++g) {
            for (int i = 0; i < particleCount; ++i) {
                particle_info.push_back(std::make_pair(g, i));
            }
        }
        
        // 随机打乱粒子顺序
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(particle_info.begin(), particle_info.end(), gen);
        
        // 计算总的网格大小
        int totalParticles = particleCount * groupCount;
        int sideLength = static_cast<int>(std::sqrt(totalParticles));
        if (sideLength * sideLength < totalParticles) {
            ++sideLength;
        }
        
        // 初始化所有粒子
        for (int idx = 0; idx < totalParticles; ++idx) {
            int g = particle_info[idx].first;
            int i = particle_info[idx].second;
            
            int row = idx / sideLength;
            int col = idx % sideLength;
            
            Particle p;
            p.position.x = col * particleInterval - init_bias_x;
            p.position.y = row * particleInterval - init_bias_y;
            p.position.z = 1.0;
            p.velocity.x = p.velocity.y = p.velocity.z = 0.0;
            p.acceleration.x = p.acceleration.y = p.acceleration.z = 0.0;
            p.force.x = p.force.y = p.force.z = 0.0;
            p.density = settings.selfDens;
            p.pressure = 0.0;
            p.hash = 0;
            p.index = idx;  // 使用全局索引
            p.group_id = g;  // 设置组ID

            std::stringstream ss;
            ss << "Group " << g << " Particle " << i;
            p.name.data = ss.str();

            particles.push_back(p);
        }
    }

    visualization_msgs::MarkerArray marker_array;
    for (const auto& p : particles) {
        visualization_msgs::Marker cube_marker;
        cube_marker.header.frame_id = "world";
        cube_marker.header.stamp = ros::Time::now();
        cube_marker.ns = "particle_cubes";
        cube_marker.id = p.index;
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;
        cube_marker.pose.position.x = p.position.x;
        cube_marker.pose.position.y = p.position.y;
        cube_marker.pose.position.z = p.position.z;
        cube_marker.pose.orientation.w = 1.0;
        cube_marker.scale.x = cube_marker.scale.y = cube_marker.scale.z = 0.1;

        // 使用新的颜色设置函数
        setParticleColor(p.group_id, cube_marker.color);
        marker_array.markers.push_back(cube_marker);
    }

    initial_pospub.publish(marker_array);
}

void SPHSystem::findNeighbors() {
    particleNeighborsTable.clear();
    nearestNeighborDistanceMap.clear();
    collisionNeighborsTable.clear();

    for (auto& particle : particles) {
        std::vector<std::pair<const Particle*, float>> neighbors;
        std::vector<const Particle*> collision_neighbors;
        double minDistance = std::numeric_limits<double>::max();

        for (auto& other : particles) {
            if (&particle == &other) continue;
                
            float dx = particle.position.x - other.position.x;
            float dy = particle.position.y - other.position.y;
            float dz = particle.position.z - other.position.z;
            float dist2 = dx * dx + dy * dy + dz * dz;
            float dist = std::sqrt(dist2);
            float h2 = settings.h * settings.h;

            if (dist2 < 4 * h2) {
                neighbors.push_back(std::make_pair(&other, dist2));
                minDistance = std::min(minDistance, static_cast<double>(dist2));
            }
            
            double dt_est = updateInterval > 0.0 ? updateInterval : 0.02;
            double collision_threshold = 2.0 * v_max * dt_est + 2 * r_aoi_scale_ * particleVisScale;
            if (dist <= collision_threshold) {
                bool same_type = (particle.isVirtual() == other.isVirtual());
                if (same_type) {
                    collision_neighbors.push_back(&other);
                }
            }
        }
        particleNeighborsTable[&particle] = neighbors;
        collisionNeighborsTable[&particle] = collision_neighbors;

        if (!neighbors.empty()) {
            nearestNeighborDistanceMap[&particle] = std::sqrt(minDistance);  // 记录实际距离（平方根）
        } else {
            nearestNeighborDistanceMap[&particle] = -1.0;  // 没有邻居
        }
    }
}

void SPHSystem::updateParticleStates(){}

void SPHSystem::update(float deltaTime) {
	if (!started) return;

    runOnGPU = false;
    updateParticles(particles, particleCount, settings, deltaTime,runOnGPU);
}

void SPHSystem::reset() {
    isInitialReceived = false;
	initParticles();
	started = false;
}

void SPHSystem::start() {
	started = true;
}

void SPHSystem::updateParticles(
    std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
    float deltaTime, const bool onGPU)
{
    if (onGPU) {
        updateParticlesGPU(
            particles, particleCount, settings, deltaTime);
    }
    else {
        updateParticlesCPU(
            particles, particleCount, settings, deltaTime);
    }
}

/// CPU update particles implementation
void SPHSystem::updateParticlesCPU(
    std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
    float deltaTime)
{   
    findNeighbors();
    updateParticleStates();
    parallelDensityAndPressures();
    parallelForces();
    computeGroupDifferentialForces();
    parallelUpdateParticlePositions(deltaTime);
    pubroscmd();
}

void SPHSystem::parallelDensityAndPressures()
{   
    for (auto& particle : particles) {

        float pDensity = 0.0;
        float neighborGrad = 0.0;

        particle.u_den.x = 0.0;
        particle.u_den.y = 0.0;
        particle.u_den.z = 0.0;

        auto& neighbors = particleNeighborsTable[&particle];
        for (auto& neighbor_pair : neighbors) {

            float dist2 = neighbor_pair.second;
            double k = std::sqrt(dist2)/h;
            double q;

            if ( k>=0 && k<=1)
            {
                q = 1-(3/2)*k*k + (3/4)*k*k*k;
            }
            else if (k>1 && k<=2)
            {
                q = (1/4)*(2-k)*(2-k)*(2-k);
            }   
            else
            {
                q = 0;
            }

            pDensity += settings.poly * q;
        }

        particle.density = pDensity + settings.selfDens;
        
        double p = -k_den  * (1/particle.density) * (std::pow(particle.density/settings.restDensity,7) -1 );
        
        for (auto& neighbor_pair : neighbors)
        {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;
            float dist = std::sqrt(dist2);

            // 计算方向向量
            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;
       
            // 归一化方向向量
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            double k = std::sqrt(dist2)/h;
            double q;

            if ( k>=0 && k<=1)
            {
                q = -3*k + (9/4)*k*k;
            }
            else if (k>1 && k<=2)
            {
                q = (-3/4)*(2-k)*(2-k);
            }   
            else
            {
                q = 0;
            }

            neighborGrad = settings.poly * h * q;

            particle.u_den.x += p * neighborGrad * dir_x;
            particle.u_den.y += p * neighborGrad * dir_y;
            particle.u_den.z += p * neighborGrad * dir_z;

        }
    }
}

void SPHSystem::parallelForces()
{
    for (auto& particle : particles) {

        particle.u_rep.x = 0.0;
        particle.u_rep.y = 0.0;
        particle.u_rep.z = 0.0;

        particle.u_fri.x = 0.0; 
        particle.u_fri.y = 0.0; 
        particle.u_fri.z = 0.0; 

        auto& neighbors = particleNeighborsTable[&particle];
        
        for (auto& neighbor_pair : neighbors) {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;

            // 计算邻居与当前粒子之间的实际距离
            float dist = std::sqrt(dist2);

            // 计算方向向量
            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;
            
            // 归一化方向向量
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            particle.u_rep.x += k_rep * 1/dist2 * dist * dir_x;
            particle.u_rep.y += k_rep * 1/dist2 * dist * dir_y;
            particle.u_rep.z += k_rep * 1/dist2 * dist * dir_z;

        }
    
        particle.u_fri.x = -k_fri * particle.velocity.x; 
        particle.u_fri.y = -k_fri * particle.velocity.y; 
        particle.u_fri.z = -k_fri * particle.velocity.z; 
    }   
}


void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{
    for (size_t i = 0; i < particles.size(); i++) {
        Particle *p = &particles[i];

        common_msgs::Acceleration acceleration;
        acceleration.x = 0.0f;
        acceleration.y = 0.0f;
        acceleration.z = 0.0f;
        
        // 根据粒子的状态来计算加速度
        switch (p->state) {
            case NULL_STATE:
                acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x + p->u_diff.x;
                acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y + p->u_diff.y;
                acceleration.z = p->u_den.z + p->u_rep.z + p->u_fri.z + p->u_diff.z;
                break;

            case NEAR_TARGET:
                acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x + p->u_diff.x;
                acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y + p->u_diff.y;
                acceleration.z = 0;
                break;
            
            default:
                std::cerr << "Error: Unknown particle state!" << std::endl;
                break;
        }
        //加速度限制
        acceleration.x = clamp(acceleration.x, a_max);
        acceleration.y = clamp(acceleration.y, a_max);
        acceleration.z = clamp(acceleration.z, a_max);  

        //update acc
        p->acceleration.x = acceleration.x;
        p->acceleration.y = acceleration.y;
        p->acceleration.z = acceleration.z;

        // 更新速度
        p->velocity.x += acceleration.x * deltaTime;
        p->velocity.y += acceleration.y * deltaTime;
        p->velocity.z += acceleration.z * deltaTime;

        //速度限制
        p->velocity.x = clamp(p->velocity.x, v_max);
        p->velocity.y = clamp(p->velocity.y, v_max);
        p->velocity.z = clamp(p->velocity.z, v_max);

    }
    
    if (avoid_enabled_) {
        avoidCollisionVelocityStep(deltaTime);
    }
    
    for (auto& particle : particles) {
        particle.position.x += particle.velocity.x * deltaTime;
        particle.position.y += particle.velocity.y * deltaTime;
        particle.position.z += particle.velocity.z * deltaTime;
        
        if (particle.position.z < 0.0) {
            particle.position.z = 0.1;
            double rebound_coeff = 0.5;
            particle.velocity.z = -particle.velocity.z * rebound_coeff;
            particle.acceleration.z = -particle.acceleration.z * rebound_coeff;
        }
    }
    
}

void SPHSystem::computeGroupDifferentialForces()
{
    for (auto& particle : particles) {
        particle.u_diff.x = 0.0;
        particle.u_diff.y = 0.0;
        particle.u_diff.z = 0.0;

        auto& neighbors = particleNeighborsTable[&particle];
        for (auto& neighbor_pair : neighbors) {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;
            float dist = std::sqrt(dist2);

            // 跳过同一个粒子
            if (dist < 1e-4) continue;

            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;

            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            float d_ij;
            // 设置吸引/排斥的目标距离
            if (particle.group_id == neighbor->group_id) {
                d_ij = d_AA;  // 同群粒子的期望距离（如 1.0）
            } else {
                d_ij = d_AB;  // 异群粒子的排斥距离（如 3.0）
            }

            // 引导力函数：类似文献中的负梯度公式
            float coeff = (dist - d_ij) + (1.0f / dist) - (d_ij / (dist * dist));
            float k_diff = k_group_sep;  // 控制引导力强度

            particle.u_diff.x += k_diff * coeff * dir_x;
            particle.u_diff.y += k_diff * coeff * dir_y;
            particle.u_diff.z += k_diff * coeff * dir_z;
        }
    }
}

bool SPHSystem::buildBarrierConstraint2D(
    const Particle& pi, const Particle& pj,
    double R, double gamma,
    Eigen::Vector2d& a_ij, double& b_ij, double& h_ij)
{
    a_ij(0) = pi.position.x - pj.position.x;
    a_ij(1) = pi.position.y - pj.position.y;

    double dist2 = a_ij.squaredNorm();
    double dist = std::sqrt(dist2);
    const double eps = 1e-9;

    if (dist < eps) {
        return false;
    }

    h_ij = dist2 - R * R;
    Eigen::Vector2d v_j_xy(pj.velocity.x, pj.velocity.y);
    b_ij = a_ij.dot(v_j_xy) - (gamma / 2.0) * h_ij;
    return true;
}

Eigen::Vector2d SPHSystem::projectOntoHalfspace2D(
    const Eigen::Vector2d& v0,
    const Eigen::Vector2d& a, double b)
{
    if (a.dot(v0) >= b) {
        return v0;
    }

    double a_norm_sq = a.squaredNorm();
    if (a_norm_sq < 1e-12) {
        return v0;
    }

    double lambda = (b - a.dot(v0)) / a_norm_sq;
    return v0 + lambda * a;
}

Eigen::Vector2d SPHSystem::solveMinChangeVelocity2D(
    const Eigen::Vector2d& v0,
    const std::vector<Eigen::Vector2d>& A,
    const std::vector<double>& B,
    double vmax)
{
    if (A.empty()) {
        double v_norm = v0.norm();
        if (v_norm <= vmax) {
            return v0;
        } else {
            return (vmax / v_norm) * v0;
        }
    }

    std::vector<Eigen::Vector2d> candidates;
    candidates.push_back(v0);

    bool v0_feasible = true;
    for (size_t i = 0; i < A.size(); ++i) {
        if (A[i].dot(v0) < B[i] - 1e-7) {
            v0_feasible = false;
            break;
        }
    }

    if (v0_feasible) {
        double v_norm = v0.norm();
        if (v_norm <= vmax) {
            return v0;
        } else {
            return (vmax / v_norm) * v0;
        }
    }

    for (size_t i = 0; i < A.size(); ++i) {
        if (A[i].dot(v0) < B[i] - 1e-7) {
            Eigen::Vector2d v_proj = projectOntoHalfspace2D(v0, A[i], B[i]);
            candidates.push_back(v_proj);
        }
    }

    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = i + 1; j < A.size(); ++j) {
            if (A[i].dot(v0) < B[i] - 1e-7 && A[j].dot(v0) < B[j] - 1e-7) {
                Eigen::Matrix2d M;
                M.row(0) = A[i].transpose();
                M.row(1) = A[j].transpose();

                Eigen::Vector2d rhs(B[i], B[j]);
                double det = M.determinant();
                if (std::abs(det) > 1e-12) {
                    Eigen::Vector2d v_intersect = M.inverse() * rhs;
                    candidates.push_back(v_intersect);
                }
            }
        }
    }

    std::vector<Eigen::Vector2d> feasible_candidates;
    for (const auto& candidate : candidates) {
        Eigen::Vector2d v_limited = candidate;
        double v_norm = candidate.norm();
        if (v_norm > vmax) {
            v_limited = (vmax / v_norm) * candidate;
        }

        bool feasible = true;
        for (size_t i = 0; i < A.size(); ++i) {
            if (A[i].dot(v_limited) < B[i] - 1e-7) {
                feasible = false;
                break;
            }
        }

        if (feasible) {
            feasible_candidates.push_back(v_limited);
        }
    }

    if (!feasible_candidates.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d best_candidate = feasible_candidates[0];

        for (const auto& candidate : feasible_candidates) {
            double dist = (candidate - v0).norm();
            if (dist < min_dist) {
                min_dist = dist;
                best_candidate = candidate;
            }
        }
        return best_candidate;
    } else {
        if (candidates.size() > 1) {
            Eigen::Vector2d avg_candidate = Eigen::Vector2d::Zero();
            for (size_t i = 1; i < candidates.size(); ++i) {
                avg_candidate += candidates[i];
            }
            avg_candidate /= (candidates.size() - 1);

            double v_norm = avg_candidate.norm();
            if (v_norm > vmax) {
                avg_candidate = (vmax / v_norm) * avg_candidate;
            }
            return avg_candidate;
        } else {
            return Eigen::Vector2d::Zero();
        }
    }
}

void SPHSystem::applyVelocityBarrierOnce(
    Particle& p, float dt,
    double R, double gamma, double r_aoi, double vmax)
{
    (void)dt;
    Eigen::Vector2d v0(p.velocity.x, p.velocity.y);
    std::vector<Eigen::Vector2d> A;
    std::vector<double> B;

    if (collisionNeighborsTable.find(&p) != collisionNeighborsTable.end()) {
        for (const Particle* neighbor : collisionNeighborsTable[&p]) {
            Eigen::Vector2d a_ij;
            double b_ij, h_ij;

            if (buildBarrierConstraint2D(p, *neighbor, R, gamma, a_ij, b_ij, h_ij)) {
                double a_norm = std::sqrt(a_ij.squaredNorm());
                if (a_norm <= r_aoi) {
                    A.push_back(a_ij);
                    B.push_back(b_ij);

                    if (h_ij < 0) {
                        double gamma_pen = c_gamma_pen_ * gamma;
                        double b_ij_pen = a_ij.dot(Eigen::Vector2d(neighbor->velocity.x, neighbor->velocity.y))
                                        - (gamma_pen / 2.0) * h_ij;
                        B.back() = b_ij_pen;
                    }
                }
            }
        }
    }

    Eigen::Vector2d v_star = solveMinChangeVelocity2D(v0, A, B, vmax);
    p.velocity.x = v_star(0);
    p.velocity.y = v_star(1);
}

void SPHSystem::avoidCollisionVelocityStep(float dt)
{
    if (dt <= 1e-6f) {
        return;
    }
    double R = particleVisScale * 1.2;
    double r_aoi = std::max(R, r_aoi_scale_ * R);
    double gamma = (1.0 - eta_) / dt;
    double vmax_local = v_max;

    for (auto& particle : particles) {
        applyVelocityBarrierOnce(particle, dt, R, gamma, r_aoi, vmax_local);
    }
}

void SPHSystem::pubroscmd() {
    visualization_msgs::MarkerArray particles_markers;
    visualization_msgs::MarkerArray state_text_markers;
    common_msgs::Swarm_particles swarm_msg;

    for (const auto& particle : particles) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.id = particle.index;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = particle.position.x;
        marker.pose.position.y = particle.position.y;
        marker.pose.position.z = particle.position.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = particleVisScale;

        // 使用新的颜色设置函数
        setParticleColor(particle.group_id, marker.color);
        particles_markers.markers.push_back(marker);

        // 添加状态文本标记
        if (state_enabled || vis_role) {
            visualization_msgs::Marker state_marker;
            state_marker.header = marker.header;
            state_marker.ns = "state_text";
            state_marker.id = particle.index + 10000;
            state_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            state_marker.action = visualization_msgs::Marker::ADD;
            state_marker.pose.position = marker.pose.position;
            state_marker.pose.position.z += 0.3;
            state_marker.scale.z = 0.2;
            state_marker.color = marker.color;  

            if (vis_role) {
                state_marker.text = "G" + std::to_string(particle.group_id) + " P" + std::to_string(particle.index);
            } else {
                state_marker.text = stateToString(particle.state);
            }
            state_text_markers.markers.push_back(state_marker);
        }

        // 添加到swarm消息
        common_msgs::Particle swarm_particle;
        swarm_particle.position = particle.position;
        swarm_particle.velocity = particle.velocity;
        swarm_particle.acceleration = particle.acceleration;
        swarm_particle.force = particle.force;
        swarm_particle.density = particle.density;
        swarm_particle.pressure = particle.pressure;
        swarm_particle.index = particle.index;
        swarm_particle.state = particle.state;
        swarm_particle.role = particle.role;
        swarm_msg.particles.push_back(swarm_particle);
    }

    // 发布odom消息
    for (const auto& particle : particles) {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        odom.child_frame_id = "particle" + std::to_string(particle.index);
        odom.pose.pose.position.x = particle.position.x;
        odom.pose.pose.position.y = particle.position.y;
        odom.pose.pose.position.z = particle.position.z;
        odom.pose.pose.orientation.w = 1.0;  // Assuming no orientation information
        odom.twist.twist.linear.x = particle.velocity.x;
        odom.twist.twist.linear.y = particle.velocity.y;
        odom.twist.twist.linear.z = particle.velocity.z;

        if (particle.index < odom_publishers.size()) {
            odom_publishers[particle.index].publish(odom);
        }
    }

    // 发布所有标记
    particles_publisher.publish(particles_markers);
    particles_publisher.publish(state_text_markers);

    // 发布 Swarm_particles 消息
    swarm_msg.header.stamp = ros::Time::now();  // 设置时间戳
    swarm_msg.header.frame_id = "world";  // 设置 frame_id
    swarm_pub.publish(swarm_msg);  // 发布消息

    // 添加时间显示文字标记
    if (started && !start_time.isZero()) {
        visualization_msgs::Marker time_marker;
        time_marker.header.frame_id = "world";
        time_marker.header.stamp = ros::Time::now();
        time_marker.ns = "time_display";
        time_marker.id = 99999;
        time_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        time_marker.action = visualization_msgs::Marker::ADD;
        time_marker.pose.position.x = 0.0;
        time_marker.pose.position.y = -17.0;
        time_marker.pose.position.z = 5.0;  // 显示在较高位置，不影响粒子运动
        time_marker.pose.orientation.w = 1.0;
        time_marker.scale.z = 1.0;  // 字体大小，设置为较大值
        time_marker.color.r = 0.0;
        time_marker.color.g = 0.0;
        time_marker.color.b = 0.0;
        time_marker.color.a = 1.0;
        
        double elapsed_time = (ros::Time::now() - start_time).toSec();
        time_marker.text = "Time: " + std::to_string(elapsed_time) + " s";
        
        visualization_msgs::MarkerArray time_marker_array;
        time_marker_array.markers.push_back(time_marker);
        particles_publisher.publish(time_marker_array);
    }
}

void SPHSystem::parallelUpdateParticleTraj() {}

SPHSystem::SPHSystem(
    size_t particleCubeWidth, const SPHSettings &settings,
    const bool &runOnGPU)
    : sph_particleCubeWidth(particleCubeWidth)
    , settings(settings)
    , runOnGPU(runOnGPU)
{
    started = false;
    initParticles();
    findNeighbors();
   
}

SPHSystem::SPHSystem()
    : sph_particleCubeWidth(0.0),
      settings(),
      started(false),
      runOnGPU(false){}

SPHSystem::~SPHSystem() {
}
