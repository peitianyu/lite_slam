#include <Eigen/Core>
#include <random>
#include <vector>
#include <memory>
#include "grid_map_manage.h"
#include "grid_map_base.h"

class Mcl
{
public:
	struct Options
	{
		// particle size
		uint particle_size = 1000;
		// parameters for motion model
		float alpha1 = 0.025f, alpha2 = 0.025f, alpha3 = 0.4f, alpha4 = 0.4f; 

		// parameters for measurement model
		float range_max = 10.0f, sigma_hit = 0.2f, z_hit = 0.8f, z_rand = 0.2f;
	};

	struct Particle
	{
		Eigen::Vector3f pose;
		float weight;
	};
public:
	Mcl(const Options& options)
	:m_options(options)
	{
		m_particles.resize(m_options.particle_size);
	}

	~Mcl();

	void LoadMap(const std::string& map_file)
	{
		m_grid_map = std::make_shared<GridMapBase>(GridMapBase::Params());
		GridMapManage::LoadGridMap(map_file, m_grid_map);
	}

	void MonteCarloLocalization(const std::vector<Eigen::Vector2f>& scan_points, const Eigen::Vector3f &pose_in_map_new, const Eigen::Vector3f &pose_in_map_old)
	{
		float total_weight  = 0.0f;

		// for each particle
		for( int i = 0; i < m_options.particle_size; i ++ ){
			// 1. draw from the motion model
			particles[i].pose = SampleMotionModelOdometry( particles[i].pose, pose_in_map_new, pose_in_map_old );

			// 2. caculate the particles' weight according to measurement model
			particles[i].weight = LikelihoodFieldRangeFinderModel( scan_points, particles[i].pose);
			
			// 3. caculate the total weight 
			total_weight += weight;
		}

		// 4. normalize the weight of each particle
		for( int i = 0; i < m_options.particle_size; i ++ ){
			particles[i].weight = particles[i].weight / total_weight;
		}
		
		// 5. resampling (重采样逻辑)
		LowVarianceSample();

		// 6. TODO ... caculate the real robot pose in world frame
		
		// 7. visualize the results by opencv
	}
private:
	void InitParticles()
	{
		std::random_device rd;  //Will be used to obtain a seed for the random number engine
	  	std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()
	  	gen.seed( rd() ); //Set random seed for random engine
		
		Eigen::Vector2i map_limit = m_grid_map->GetMapLimit();
		std::uniform_real_distribution<float> x( static_cast<float>(map_limit(0, 0)), static_cast<float>(map_limit(0, 1))) ;
        std::uniform_real_distribution<float> y( static_cast<float>(map_limit(1, 0)), static_cast<float>(map_limit(1, 1)));
        std::uniform_real_distribution<float> theta(-M_PI, M_PI);
	
		for(Particle& p:m_particles){
			p = Particle(Eigen::Vector3f(x(gen), y(gen), theta(gen)),  1.0f / static_cast<float>(m_options.particle_size));
		}
	}

	const Eigen::Vector3f SampleMotionModelOdometry( const Eigen::Vector3f &p, const Eigen::Vector3f &p_new, const Eigen::Vector3f &p_old)
	{
		float delta_rot1 = ::atan2( p_new(1) - p_old(1), p_new(0) - p_old(0) ) - p_old(2);
		float delta_trans = ::sqrt( ( p_old(0) - p_new(0) ) * ( p_old(0) - p_new(0) ) + ( p_old(1) - p_new(1) ) * ( p_old(1) - p_new(1) ) );
		float delta_rot2 = p_new(2) - p_old(2) - delta_rot1;

		float delta_rot1_hat = delta_rot1 - SampleStandardNormalDistribution( m_options.alpha1 * delta_rot1 + m_options.alpha2 * delta_rot2 );
        float delta_trans_hat = delta_trans - SampleStandardNormalDistribution( m_options.alpha3 * delta_trans + m_options.alpha4 * ( delta_rot1 + delta_rot2 ) );
        float delta_rot2_hat = delta_rot2 - SampleStandardNormalDistribution( m_options.alpha1 * delta_rot2 + m_options.alpha2 * delta_trans );

		return Eigen::Vector3f(p(0) + delta_trans_hat * ::cos( p(2) + delta_rot1_hat ),
							   p(1) + delta_trans_hat * ::sin( p(2) + delta_rot1_hat ),
							   p(2) + delta_rot1_hat + delta_rot2_hat);
	}

	const float SampleStandardNormalDistribution(const float var)
	{
		float sum = 0;
		for (int i = 0;i < 12; i++)
			sum += (rand() - RAND_MAX / 2.0) / (float)RAND_MAX * 2.0;

		return (var / 6.0) * sum;
	}

	const float LikelihoodFieldRangeFinderModel(const std::vector<Eigen::Vector2f>& scan_points, const Eigen::Vector3f &pose_in_world)
	{
		float z_hit_denom = 2.0f * m_options.sigma_hit * m_options.sigma_hit;
    	float z_rand_mult = 1.0f / m_options.range_max;
	
		float p = 1.0f;
		for(Eigen::Vector2f& point: scan_points)
		{
			Eigen::Vector2f point_in_map = m_grid_map->WorldToMapFloat(PointTransform(pose_in_world, point));

			// float dist = m_kdtree->GetMapCellDist(point_in_map.cast<int>()); // kd_tree 实现

			float pz = 0.0f;
			pz += m_options.z_hit * ::exp( -( dist * dist ) / z_hit_denom );
			pz += m_options.z_rand * z_rand_mult;

			assert( pz <= 1.0f );
      		assert( pz >= 0.0f );
			
			// TODO ... May be changed according to different situation
			p += pz * pz;
		}

		return p;
	}

	Eigen::Vector2f PointTransform(const Eigen::Vector3f& pose_in_world, const Eigen::Vector2f& new_point)
	{
		float dx = new_point(0) - pose_in_world(0);
		float dy = new_point(1) - pose_in_world(1);

		return Eigen::Vector2f(
				dx * std::cos(pose_in_world(2)) + dy * std::sin(pose_in_world(2)),
			   -dx * std::sin(pose_in_world(2)) + dy * std::cos(pose_in_world(2)));
	}

	void LowVarianceSample()
	{
		std::vector<Particle> particles_temp( particles, particles + m_options.particle_size );

		float random = ( rand() / (float)RAND_MAX) * (1.0f / (float)m_options.particle_size ); 

		float c = particles[0].weight;

		int index = 0;

		for (int m = 0; m < m_options.particle_size; m ++) {
			float u = random + (float)m / (float)m_options.particle_size;
 		
			while (u > c && index < m_options.particle_size - 1){ 
				index++;				
				c += particles_temp[index].weight;	
			}
			particles[m] = particles_temp[index]; 	 
			particles[m].weight = 1.0f / m_options.particle_size;
		}	
	}



private:
	Options m_options;
	std::vector<Particle> m_particles;
	std::shared_ptr<GridMapBase> m_grid_map;
};