#include"ndt_match.h"


NdtMatch::NdtMatch(const Options &options)
:m_options(options)
{
    m_grids.resize(options.col_num * options.row_num + 1);
}

bool NdtMatch::Match( const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan, Eigen::Vector3f &robot_pose)
{
    if( first_scan.empty() || second_scan.empty() )
        {return false;}

    CalculateNdtGrid( first_scan );
    
    Eigen::Vector3f pose_delta( 0.0f, 0.0f, 0.0f );
    for(uint iteration = 0; iteration < m_options.max_iterations; iteration ++ )     
        {EstimateTransformationOnce( second_scan, pose_delta );}
    
    NormalizePose( pose_delta );

    if( fabs(pose_delta[0]) < 1 && fabs(pose_delta[1]) < 1 ){
        robot_pose += pose_delta;
    }
    return true;
}   

void NdtMatch::CalculateNdtGrid( const std::vector<Eigen::Vector2f> &scan )
{
    memset( m_grids.data(), 0, m_grids.size() * sizeof( m_grids[0] ) );     

    for( Eigen::Vector2f point: scan){
        int index = PointToGrid( point );   
        m_grids[index].mean += point;
        m_grids[index].points.push_back( point );
    }   

    for( size_t i = 0; i < m_grids.size(); i ++ ){
        if( m_grids[i].points.size() >= 3 ){
            m_grids[i].mean = m_grids[i].mean / static_cast<float>( m_grids[i].points.size() );

            for( Eigen::Vector2f point : m_grids[i].points )
                {m_grids[i].covarince += ( point - m_grids[i].mean ) * ( point - m_grids[i].mean ).transpose();}
            m_grids[i].covarince /= static_cast<float>( m_grids[i].points.size()  - 1 );
        }
        else {
            m_grids[i].covarince << std::numeric_limits<float>::max(), 0, 0, std::numeric_limits<float>::max();
        }
    }
    
}

void NdtMatch::GetHessianDerived( const std::vector<Eigen::Vector2f> &scan, const Eigen::Vector3f &p, Eigen::Matrix3f &H, Eigen::Vector3f &b )
{
    for( Eigen::Vector2f point: scan ){
        Eigen::Vector2f point_in_first_frame = PointTransform( point, p );                
        int index = PointToGrid( point_in_first_frame );

        Eigen::Vector2f error = point_in_first_frame - m_grids[index].mean;
        Eigen::Matrix2f sigma_inverse = ( m_grids[index].covarince ).inverse(); 

        Eigen::Matrix<float, 2, 3> jacobian;
        jacobian << 1, 0,  -point[0] * ::sin( p[2] ) - point[1] * ::cos( p[2] ),
                    0, 1,   point[0] * ::cos( p[2] ) - point[1] * ::sin( p[2] );

        b += ( error.transpose() * sigma_inverse * jacobian ).transpose();
        H += jacobian.transpose() * sigma_inverse * jacobian;           
    }
}   

void NdtMatch::EstimateTransformationOnce( const std::vector<Eigen::Vector2f> &scan, Eigen::Vector3f &p )            
{
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f b = Eigen::Vector3f::Zero();
    GetHessianDerived( scan, p, H, b );
    
    p -= H.inverse() * b; 
}
   
const int NdtMatch::PointToGrid( const Eigen::Vector2f &point ) const
{
    Eigen::Vector2i grid_index = Eigen::Vector2i::Zero();
    grid_index(0) = static_cast<int>(point(0) / (m_options.laser_dist / (m_options.col_num / 2)));
    grid_index(1) = static_cast<int>(point(1) / (m_options.laser_dist / (m_options.row_num / 2)));
    return ((m_options.row_num / 2 - grid_index(1)) * m_options.col_num + grid_index(0) + m_options.row_num / 2);
}

const Eigen::Vector2f NdtMatch::PointTransform( const Eigen::Vector2f &point, const Eigen::Vector3f &d_pose ) const
{
    return Eigen::Vector2f(point(0) * cos(d_pose(2)) - point(1) * sin(d_pose(2)) + d_pose(0),
                            point(0) * sin(d_pose(2)) + point(1) * cos(d_pose(2)) + d_pose(1));
}

void NdtMatch::NormalizePose(Eigen::Vector3f &pose)
{
    while(pose(2) > M_PI) pose(2) -= 2 * M_PI;
    while(pose(2) < -M_PI) pose(2) += 2 * M_PI;
}