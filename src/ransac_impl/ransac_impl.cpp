#include "ransac_impl.h"


std::unordered_set<int> Ransac::RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
								 double maxIterations, double distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	// std::cout << maxIterations << std::endl;
	while(maxIterations-- ){
		std::unordered_set<int> inliers;
		while(inliers.size() < 2){
			inliers.insert(rand()%cloud->points.size());
		}
		float x1 = cloud->points[*inliers.begin()].x;
		float x2 = cloud->points[*++inliers.begin()].x;
		float y1 = cloud->points[*inliers.begin()].y;
		float y2 = cloud->points[*++inliers.begin()].y;

		float a = y1 - y2;
		float b = x2 - x1;
		float c = (x1 * y2) - (x2 * y1);

		for(int i = 0; i < cloud->size(); i++){
			if(inliers.find(i) != inliers.end()){
				continue;
			}
			
			float x3 = cloud->points[i].x;
			float y3 = cloud->points[i].y;

			float dist = std::fabs((a*x3) + (b*y3) + c)/sqrt((a*a) + (b*b));
			std::cout << x1 << "  " << x2 << "  " << y1 << "  " << y2 << "  " << a << "  " << b << "  " << c << "  " << x3 << "  " << y3 << "  " << dist << std::endl;
			if(dist < distanceTol){
				inliers.insert(i);
				std::cout << x1 << "  " << x2 << "  " << y1 << "  " << y2 << "  " << a << "  " << b << "  " << c << "  " << x3 << "  " << y3 << "  " << dist << "   " << distanceTol << std::endl;
			}

		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}
	
	return inliersResult;

}

std::unordered_set<int> Ransac::RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    std::unordered_set<int> inliersResult;
	// srand(time(NULL));
	
	while(Ransac::maxIteration-- ){
		std::unordered_set<int> inliers;
		while(inliers.size() < 3){
			inliers.insert(rand()%cloud->points.size());
		}
		std::array<float, 3> p1 = {cloud->points[*inliers.begin()].x, cloud->points[*inliers.begin()].y, cloud->points[*inliers.begin()].z};
		std::array<float, 3> p2 = {cloud->points[*++inliers.begin()].x, cloud->points[*++inliers.begin()].y, cloud->points[*++inliers.begin()].z};
        std::array<float, 3> p3 = {cloud->points[*++(++inliers.begin())].x, cloud->points[*++(++inliers.begin())].y, cloud->points[*++(++inliers.begin())].z};
        
        std::array<float, 3> v1 = {(p2.at(0)-p1.at(0)), (p2.at(1)-p1.at(1)), (p2.at(2)-p1.at(2))};
        std::array<float, 3> v2 = {(p3.at(0)-p1.at(0)), (p3.at(1)-p1.at(1)), (p3.at(2)-p1.at(2))};
        
		std::array<float, 3> xprod;
        float a = xprod.at(0) = (v1.at(1) * v2.at(2)) - (v1.at(2) * v2.at(1)) ;
        float b = xprod.at(1) = (v1.at(2) * v2.at(0)) - (v1.at(0) * v2.at(2)) ;
        float c = xprod.at(2) = (v1.at(0) * v2.at(1)) - (v1.at(1) * v2.at(0)) ;
        float d = -(a * p1.at(0) + b * p1.at(1) + c * p1.at(2));

		for(int i = 0; i < cloud->size(); i++){
			if(inliers.find(i) != inliers.end()){
				continue;
			}
			
			std::array<float, 3> p4 = {cloud->points[i].x, cloud->points[i].y,cloud->points[i].z};
			
			float dist = std::fabs((a*p4.at(0)) + (b*p4.at(1)) + (c*p4.at(2)) + d)/sqrt((a*a) + (b*b) + (c*c));
			std::cout << p1.at(0) << "  " << p1.at(1) << "  " << p1.at(2) << "  " 
                      << p2.at(0) << "  " << p2.at(1) << "  " << p2.at(2) << "  "  
                      << p3.at(0) << "  " << p3.at(1) << "  " << p3.at(2) << "  " 
                      << a << "  " << b << "  " << c << "  " << d << "  " 
                      << p4.at(0) << "  " << p4.at(1) << "  " << p4.at(2) << "  " << dist << "  " << Ransac::threshold << std::endl;
			if(dist < Ransac::threshold){
				inliers.insert(i);
				// std::cout << x1 << "  " << x2 << "  " << y1 << "  " << y2 << "  " << a << "  " << b << "  " << c << "  " << x3 << "  " << y3 << "  " << dist << "   " << distanceTol << std::endl;
			}

		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}
	
	return inliersResult;


}
    