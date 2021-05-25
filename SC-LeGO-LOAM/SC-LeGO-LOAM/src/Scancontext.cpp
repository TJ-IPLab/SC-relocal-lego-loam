#include "Scancontext.h"

// namespace SC2
// {
#include "opencv2/opencv.hpp"
#include <tuple>

const int scale = 10;
const float maxz = 23.7;

cv::Mat SCManager::createSci (Eigen::MatrixXd &scsc)
{
    int a = scsc.rows() * scale, b = scsc.cols() * scale;
    cv::Mat sc = cv::Mat(a, b, CV_8UC1);

    //relative height to determine the pixel
    /**
    double maxz=0;
    for(int i = 0; i < scsc.rows(); i++)
    for(int j = 0; j < scsc.cols(); j++)
    {
        if(scsc(i,j)>maxz)
        maxz=scsc(i,j);
    }
**/

    for (int i = 0; i < scsc.rows(); i++)
    {
        for (int j = 0; j < scsc.cols(); j++)
        {
            uchar pixel = floor(255 * scsc(i, j) / maxz);
            for (int u = i * scale; u < (i + 1) * scale; u++)
                for (int v = j * scale; v < (j + 1) * scale; v++)
                {
                    sc.at<uchar>(u, v) = pixel;
                }
        }
    }
    cv::Mat sci;
    cv::applyColorMap(sc, sci, cv::COLORMAP_JET);
    
    return sci;
}

cv::Mat SCManager::addAxes(cv::Mat &sci,std::string title)
{
    DrawAxes drawAxes;      
    cv::Mat ImageAddAxes = cv::Mat(sci.rows+100, sci.cols+70, CV_8UC3, cv::Scalar(255,255,255));
    drawAxes.InputFigure(sci, ImageAddAxes);
    std::string ylabel = "Ring(radius:0-80m)";
	std::string xlabel = "Sector(anti-clock)";
	std::string title_name = title;
	drawAxes.DrawLabel_Y(ylabel, 20, 0, 4, cv::Scalar(0, 0, 0));
	drawAxes.DrawLabel_X(xlabel, 0, 60, 6, cv::Scalar(0, 0, 0));
	drawAxes.DrawTitle(title_name);
    return ImageAddAxes;
}

void SCManager::show_diff(std::vector<std::tuple<double, int, int, cv::Mat>> diff_vector/*1.distance 2.index 3.align 4.diff_image*/)
{
    for (int index = 0; index < 3; index++)
    {
        std::cout << "diff_vector.first(distance): " << std::get<0>(diff_vector[index]) << std::endl;
        std::cout << "diff_vector.second(index): " << std::get<1>(diff_vector[index]) << std::endl;
        std::cout << "diff_vector.third(align): " << std::get<2>(diff_vector[index]) << std::endl;

        // Eigen::MatrixXd scShift_candi = circshift(polarcontexts_[std::get<1>(diff_vector[index])],std::get<2>(diff_vector[index]));
        // Eigen::MatrixXd diff = scShift_candi - sc;
        // cv::Mat sciForWhitePoint_candi = createSci(diff);
        // cv::Mat sciForWhitePointAddAxes_candi = addAxes(sciForWhitePoint_candi,"     diff between sc and " + std::to_string(index));

        cv::Mat shown_diff_image = addAxes(std::get<3>(diff_vector[index]), "diff between sc and " + std::to_string(index));
        cv::imshow("diff between sc and " + std::to_string(index), shown_diff_image);
        cv::moveWindow("diff between sc and " + std::to_string(index), 50, 50 + 400 * (index));
        cv::waitKey(1);
    }
}

MatrixXd SCManager::makeTransformScancontext( pcl::PointCloud<SCPointType> & _scan_down, int trans_x, int trans_y )
{
    TicToc t_making_desc;
    
    int num_pts_scan_down = _scan_down.points.size();

    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x - trans_x; 
        pt.y = _scan_down.points[pt_idx].y - trans_y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        if(pt.z < 0)
        continue;

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // taking maximum z 
        if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
            desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
}

void coreImportTest (void)
{
    cout << "scancontext lib is successfully imported." << endl;
} // coreImportTest


float rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
}

float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}


float xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
} // xy2theta


MatrixXd circshift( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift

Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> circshift( Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> shifted_mat = _mat;
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift

std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} // eig2stdvec


double SCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);
        
        if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
            continue; // don't count this sector pair. 

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC

double SCManager::distDirectSC(MatrixXd &_sc1, Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> &_sc2_downres,
                               cv::Mat &diff_image)
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;

    int a = _sc1.rows() * scale, b = _sc1.cols() * scale;
    cv::Mat sc_diff = cv::Mat(a, b, CV_8UC1);

    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = col_sc1;
        for (int row_idx = 0; row_idx < col_sc2.rows(); row_idx++)
        {
            float ref = _sc1(row_idx, col_idx);
            std::vector<float> v3 = _sc2_downres(row_idx, col_idx);
            auto i = std::min_element(begin(v3), end(v3), [=](float x, float y) {
                return abs(x - ref) < abs(y - ref);
            });
            col_sc2(row_idx) = v3[std::distance(begin(v3), i)];
        }

        if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
            continue; // don't count this sector pair. 

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        // uchar pixel = floor(255 * (1 - sector_similarity));
        for (int row_idx = 0; row_idx < _sc1.rows(); row_idx++)
        {
            uchar pixel = floor(255 * abs(col_sc1(row_idx) - col_sc2(row_idx)) / maxz);
            for (int u = row_idx * scale; u < (row_idx + 1) * scale; u++)
            {
                for (int v = col_idx * scale; v < (col_idx + 1) * scale; v++)
                {
                    sc_diff.at<uchar>(u, v) = pixel;
                }
            }
        }

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    cv::applyColorMap(sc_diff, diff_image, cv::COLORMAP_JET);

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC

int SCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey


std::pair<double, int> SCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    // 1. fast align using variant key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext

std::pair<double, int> SCManager::distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2,
                                                         Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> &_sc2_down, 
                                                         std::pair<double, int> &result_res_1,
                                                         cv::Mat &diff_image)
{
    // 1. fast align using variant key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    int argmin_shift_downres = 0;
    double min_sc_dist_downres = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> sc2_shifted_downres = circshift(_sc2_down, num_shift);
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        double cur_sc_dist_downres = distDirectSC( _sc1, sc2_shifted_downres, diff_image );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
        if( cur_sc_dist_downres < min_sc_dist_downres )
        {
            argmin_shift_downres = num_shift;
            min_sc_dist_downres = cur_sc_dist_downres;
        }
    }
    result_res_1.first = min_sc_dist_downres;
    result_res_1.second = argmin_shift_downres;
    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext

MatrixXd SCManager::makeScancontext( pcl::PointCloud<SCPointType> & _scan_down )
{
    TicToc t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        if (pt.z < 0)
            continue;

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // taking maximum z 
        if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
            desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
} // SCManager::makeScancontext


MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
} // SCManager::makeRingkeyFromScancontext


MatrixXd SCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} // SCManager::makeSectorkeyFromScancontext


Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> SCManager::down_resolution( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: down resolution 3-dimensions vector
    */
    Eigen::Matrix<std::vector<float>, 1, 1> downres_key1;
    // Eigen::Matrix<std::vector<float>, _desc.rows(), _desc.cols()> downres_key;
    Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> downres_key;
    downres_key.resize(_desc.rows(), _desc.cols());
    // Eigen::MatrixXd downres_key(_desc.rows(), _desc.cols());
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
        {
            float cur = _desc(row_idx, col_idx);
            float left = _desc(row_idx, (col_idx - 1 + _desc.cols()) % _desc.cols());
            float right = _desc(row_idx, (col_idx + 1 + _desc.cols()) % _desc.cols());
            // MatrixXd single_grid(1, 3);
            // single_grid(0, 0) = left;
            // single_grid(0, 1) = cur;
            // single_grid(0, 2) = right;
            // downres_key(row_idx, col_idx) = std::max(std::max(cur, left), right);
            // downres_key(row_idx, col_idx) = single_grid;
            downres_key(row_idx, col_idx).push_back(left);
            downres_key(row_idx, col_idx).push_back(cur);
            downres_key(row_idx, col_idx).push_back(right);
        }
    }

    return downres_key;
} // SCManager::down_resolution


void SCManager::makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down )
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1 
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

    polarcontexts_.push_back( sc ); 
    Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> multires_1_sc = down_resolution(sc);
    polarcontexts_multires_1_.push_back( multires_1_sc );
    polarcontext_invkeys_.push_back( ringkey );
    polarcontext_vkeys_.push_back( sectorkey );
    polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );

    // cout <<polarcontext_vkeys_.size() << endl;

} // SCManager::makeAndSaveScancontextAndKeys


std::pair<int, float> SCManager::detectLoopClosureID ( void )
{
    int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
    auto curr_desc = polarcontexts_.back(); // current observation (query)

    /* 
     * step 1: candidates from ringkey tree_
     */
    if( polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result; // Early return 
    }

    // tree_ reconstruction (not mandatory to make everytime)
    if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {
        TicToc t_tree_construction;

        polarcontext_invkeys_to_search_.clear();
        polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT ) ;

        polarcontext_tree_.reset(); 
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;
        
    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");

    /* 
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */
    TicToc t_calc_dist;   
    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {
        MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }
    t_calc_dist.toc("Distance calc");

    /* 
     * loop threshold check
     */
    if( min_dist < SC_DIST_THRES )
    {
        loop_id = nn_idx; 
    
        // std::cout.precision(3); 
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }
    else
    {
        std::cout.precision(3); 
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result {loop_id, yaw_diff_rad};

    return result;

} // SCManager::detectLoopClosureID

void SCManager::setThres( double thres )
{
    SC_DIST_THRES = thres;

} // SCManager::setThres

bool cmp(const std::tuple<double, int, int, cv::Mat> a, const std::tuple<double, int, int, cv::Mat> b) {
    return std::get<0>(a) < std::get<0>(b); //自定义的比较函数
}

std::pair<int, float> SCManager::detectRelocalID( pcl::PointCloud<SCPointType> & _scan_down, std::vector<int> &kd_candi )
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1 
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

    int loop_id { -1 }; // init with -1

    auto curr_key = polarcontext_invkey_vec; // current observation (query)
    auto curr_desc = sc; // current observation (query)
    
    // std::cout << "start detectRelocalID ... construct query finished" << std::endl;

    /* 
     * step 1: candidates from ringkey tree_
     */
        
    // tree_ reconstruction (not mandatory to make everytime)
    TicToc t_tree_construction;

    polarcontext_invkeys_to_search_.clear();
    polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end()) ;

    polarcontext_tree_.reset(); 
    polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ );
    // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
    t_tree_construction.toc("Tree construction");
    
    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );
    // std::cout << "start detectRelocalID ... candidates from ringkey tree inited" << std::endl;

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    // std::cout << "start detectRelocalID ... ready to findNeighbors" << std::endl;
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");
    // std::cout << "start detectRelocalID ... candidates from ringkey tree finished" << std::endl;

    /* 
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */
    TicToc t_calc_dist;
    std::pair<double, int> sc_dist_result_res_1;
    std::vector<std::tuple<double, int, int, cv::Mat>> results;//first(distance) second(index) third(align) fourth(diff_image)
    for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
    {
        std::cout << "calculating diff between cur and " << candidate_indexes[candidate_iter_idx] << std::endl;
        kd_candi.push_back(candidate_indexes[candidate_iter_idx]);
        cv::Mat one_diff_image;
        MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
        Eigen::Matrix<std::vector<float>, Dynamic, Dynamic> polarcontext_candidate_multires_1 = polarcontexts_multires_1_[ candidate_indexes[candidate_iter_idx] ];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate );

        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        // use multires-1 to relocal

        // sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate,
        //                                                                polarcontext_candidate_multires_1,
        //                                                                sc_dist_result_res_1,
        //                                                                one_diff_image);
        // candidate_dist = sc_dist_result_res_1.first;
        // candidate_align = sc_dist_result_res_1.second;
        // results.emplace_back(std::tuple<double, int, int, cv::Mat>{candidate_dist, candidate_indexes[candidate_iter_idx],
        //                                                            candidate_align, one_diff_image});

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;
            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }
    t_calc_dist.toc("Distance calc");
    if (results.size() != 0)
    {
        std::sort(results.begin(), results.end(), cmp);
        show_diff(results);
    }
    // std::cout << "start detectRelocalID ... pairwise distance calculate finished" << std::endl;

    /* 
     * loop threshold check
     */

//--------------------------------------------transform------------------------------------------------------------
/**    
    cout<<"save is start"<<endl;
    
    std::string path = "/home/zeng/catkin_ws/data/original.pcd";
    pcl::io::savePCDFileASCII(path, _scan_down);

    for(int i = 0; i <21; ++i)
    {
        Eigen::MatrixXd sc_transform_x = makeTransformScancontext(_scan_down, i,0);
        Eigen::MatrixXd sc_transform_y = makeTransformScancontext(_scan_down, 0,i);
        Eigen::MatrixXd sc_transform_xy = makeTransformScancontext(_scan_down, i,i);
        cv::Mat sci_transform_x = createSci(sc_transform_x);
        cv::Mat sci_transform_y = createSci(sc_transform_y);
        cv::Mat sci_transform_xy = createSci(sc_transform_xy);
        char name_x[100],name_y[100],name_xy[100];
        sprintf(name_x,"/home/zeng/catkin_ws/data/transform_x/x+%d.jpg",i);
        sprintf(name_y,"/home/zeng/catkin_ws/data/transform_y/y+%d.jpg",i);
        sprintf(name_xy,"/home/zeng/catkin_ws/data/transform_xy/x+%d,y+%d.jpg",i,i);
        cv::imwrite(name_x,sci_transform_x);
        cv::imwrite(name_y,sci_transform_y);
        cv::imwrite(name_xy,sci_transform_xy);
    }
    cout<<"save is finish"<<endl;
**/
//-----------------------------------------transform------------------------------------------------------------

// --------------------------------------------------create sc image--------------------------------------

    cv::Mat sciForRedPoint = createSci(sc);
    cv::Mat sciForRedPointAddAxes = addAxes(sciForRedPoint,"     red point");

    Eigen::MatrixXd scShift = circshift(polarcontexts_[nn_idx],nn_align);
    cv::Mat sciForWhitePoint = createSci(scShift);
    cv::Mat sciForWhitePointAddAxes = addAxes(sciForWhitePoint,"     white point");

    cv::imshow("red point",sciForRedPointAddAxes);
    cv::imshow("white point",sciForWhitePointAddAxes);
    cv::moveWindow("red point", 800, 50);
    cv::moveWindow("white point", 1600, 50);
    cv::waitKey(1);

// --------------------------------------------------create sc image--------------------------------------

    if( min_dist < SC_DIST_THRES )
    {
        loop_id = nn_idx; 
    
        // std::cout.precision(3); 
        if (results.size() == 0)
        {
            cout << "[found]         Nearest distance: " << min_dist << " between current pointcloud and " << nn_idx << "." << endl;
            cout << "[found]         yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
        }
        else
        {
            cout << "[MR1 found]     Nearest distance: " << sc_dist_result_res_1.first << " between current pointcloud and " << nn_idx << "." << endl;
            cout << "[MR1 found]     yaw diff: " << sc_dist_result_res_1.second * PC_UNIT_SECTORANGLE << " deg." << endl;
        }
        cout << "---------------------------------- complete: " << relocal_count << " ----------------------------------"<<endl;
        relocal_count++;
    }
    else
    {
        // std::cout.precision(3); 
        if (results.size() == 0)
        {
            cout << "[Not found]     Nearest distance: " << min_dist << "between current pointcloud and " << nn_idx << "." << endl;
            cout << "[Not found]     yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
        }
        else
        {
            cout << "[MR1 Not found] Nearest distance: " << sc_dist_result_res_1.first << " between current pointcloud and " << nn_idx << "." << endl;
            cout << "[MR1 Not found] yaw diff: " << sc_dist_result_res_1.second * PC_UNIT_SECTORANGLE << " deg." << endl;
        }
        cout << "------------------------------------------------------" << relocal_count << "----------------------------------"<<endl;
        relocal_count++;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result {loop_id, yaw_diff_rad};

    return result;

} // SCManager::detectRelocalID

// } // namespace SC2