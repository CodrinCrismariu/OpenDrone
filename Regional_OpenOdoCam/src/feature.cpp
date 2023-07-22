#include "feature.h"
#include "bucket.h"

void featureDetectionFast(cv::Mat image, std::vector<cv::Point2f>& points)  
{   
//uses FAST as for feature dection, modify parameters as necessary
  std::vector<cv::KeyPoint> keypoints;
  int fast_threshold = 30;
  bool nonmaxSuppression = true;
  cv::FAST(image, keypoints, fast_threshold, nonmaxSuppression);
  cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                          std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                          std::vector<cv::Point2f>& points0_return,
                          std::vector<uchar>& status0, std::vector<uchar>& status1,
                          std::vector<uchar>& status2, std::vector<uchar>& status3,
                          std::vector<int>& ages){
  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  for (int i = 0; i < ages.size(); ++i)
  {
     ages[i] += 1;
  }

  int indexCorrection = 0;
  for( int i=0; i<status3.size(); i++)
     {  cv::Point2f pt0 = points0.at(i- indexCorrection);
        cv::Point2f pt1 = points1.at(i- indexCorrection);
        cv::Point2f pt2 = points2.at(i- indexCorrection);
        cv::Point2f pt3 = points3.at(i- indexCorrection);
        cv::Point2f pt0_r = points0_return.at(i- indexCorrection);
        
        if ((status3.at(i) == 0)||(pt3.x<0)||(pt3.y<0)||
            (status2.at(i) == 0)||(pt2.x<0)||(pt2.y<0)||
            (status1.at(i) == 0)||(pt1.x<0)||(pt1.y<0)||
            (status0.at(i) == 0)||(pt0.x<0)||(pt0.y<0))   
        {
          if((pt0.x<0)||(pt0.y<0)||(pt1.x<0)||(pt1.y<0)||(pt2.x<0)||(pt2.y<0)||(pt3.x<0)||(pt3.y<0))    
          {
            status3.at(i) = 0;
          }
          points0.erase (points0.begin() + (i - indexCorrection));
          points1.erase (points1.begin() + (i - indexCorrection));
          points2.erase (points2.begin() + (i - indexCorrection));
          points3.erase (points3.begin() + (i - indexCorrection));
          points0_return.erase (points0_return.begin() + (i - indexCorrection));

          ages.erase (ages.begin() + (i - indexCorrection));
          indexCorrection++;
        }

     }  
}

void circularMatching(cv::Mat img_l_0, cv::Mat img_r_0, cv::Mat img_l_1, cv::Mat img_r_1,
                      std::vector<cv::Point2f>& points_l_0, std::vector<cv::Point2f>& points_r_0,
                      std::vector<cv::Point2f>& points_l_1, std::vector<cv::Point2f>& points_r_1,
                      std::vector<cv::Point2f>& points_l_0_return,
                      FeatureSet& current_features) { 
  
  //this function automatically gets rid of points for which tracking fails

  std::vector<float> err;                    
  cv::Size winSize=cv::Size(21,21);                                                                                             
  cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

  std::vector<uchar> status0;
  std::vector<uchar> status1;
  std::vector<uchar> status2;
  std::vector<uchar> status3;

  clock_t tic = clock();
  calcOpticalFlowPyrLK(img_l_0, img_r_0, points_l_0, points_r_0, status0, err, winSize, 3, termcrit, 0, 0.001);
  calcOpticalFlowPyrLK(img_r_0, img_r_1, points_r_0, points_r_1, status1, err, winSize, 3, termcrit, 0, 0.001);
  calcOpticalFlowPyrLK(img_r_1, img_l_1, points_r_1, points_l_1, status2, err, winSize, 3, termcrit, 0, 0.001);
  calcOpticalFlowPyrLK(img_l_1, img_l_0, points_l_1, points_l_0_return, status3, err, winSize, 3, termcrit, 0, 0.001);
  clock_t toc = clock();
  std::cerr << "calcOpticalFlowPyrLK time: " << float(toc - tic)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;


  deleteUnmatchFeaturesCircle(points_l_0, points_r_0, points_r_1, points_l_1, points_l_0_return,
                        status0, status1, status2, status3, current_features.ages);

  // std::cout << "points : " << points_l_0.size() << " "<< points_r_0.size() << " "<< points_r_1.size() << " "<< points_l_1.size() << " "<<std::endl;
}

std::vector < cv::Mat > createGrid(cv::Mat img, int grid_height, int grid_width) {

  cv::Size s = img.size();
  double HEIGHT_RES = s.height;
  double WIDTH_RES = s.width;

  cv::Mat uImg = img;

  std::vector < cv::Mat > ans;
  double height_ratio = HEIGHT_RES / grid_height;
  double width_ratio = WIDTH_RES / grid_width;
  for(int i = 0; i < grid_height; i++)
    for(int j = 0; j < grid_width; j++) {
        int x_0 = std::max(height_ratio * i - HEIGHT_RES * 0.05, 0.0);
        int x_1 = std::min(height_ratio * (i + 1) + HEIGHT_RES * 0.05, HEIGHT_RES);

        int y_0 = std::max(width_ratio * j - WIDTH_RES * 0.05, 0.0);
        int y_1 = std::min(width_ratio * (j + 1) + WIDTH_RES * 0.05, WIDTH_RES);

        ans.push_back(cv::Mat(uImg, cv::Range(x_0, x_1), cv::Range(y_0, y_1)));
    }

  return ans;

}

void opencl_circularMatching(cv::Mat Img_l_0, cv::Mat Img_r_0, cv::Mat Img_l_1, cv::Mat Img_r_1,
                      std::vector<cv::Point2f>& points_l_0, std::vector<cv::Point2f>& points_r_0,
                      std::vector<cv::Point2f>& points_l_1, std::vector<cv::Point2f>& points_r_1,
                      std::vector<cv::Point2f>& points_l_0_return,
                      FeatureSet& current_features) { 
  
  //this function automatically gets rid of points for which tracking fails

  std::vector<float> err;                    
  cv::Size winSize=cv::Size(21,21);                                                                                             
  cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

  std::vector<uchar> status0;
  std::vector<uchar> status1;
  std::vector<uchar> status2;
  std::vector<uchar> status3;

  cv::UMat img_l_0 = Img_l_0.getUMat(cv::ACCESS_RW);
  cv::UMat img_l_1 = Img_l_1.getUMat(cv::ACCESS_RW);
  cv::UMat img_r_0 = Img_r_0.getUMat(cv::ACCESS_RW);
  cv::UMat img_r_1 = Img_r_1.getUMat(cv::ACCESS_RW);

  clock_t tic = clock();
  calcOpticalFlowPyrLK(img_l_0, img_r_0, points_l_0, points_r_0, status0, err, winSize, 3, termcrit, 0, 0.001);
  calcOpticalFlowPyrLK(img_r_0, img_r_1, points_r_0, points_r_1, status1, err, winSize, 3, termcrit, 0, 0.001);
  calcOpticalFlowPyrLK(img_r_1, img_l_1, points_r_1, points_l_1, status2, err, winSize, 3, termcrit, 0, 0.001);
  calcOpticalFlowPyrLK(img_l_1, img_l_0, points_l_1, points_l_0_return, status3, err, winSize, 3, termcrit, 0, 0.001);
  clock_t toc = clock();
  std::cerr << "calcOpticalFlowPyrLK time: " << float(toc - tic)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;


  deleteUnmatchFeaturesCircle(points_l_0, points_r_0, points_r_1, points_l_1, points_l_0_return,
                        status0, status1, status2, status3, current_features.ages);

  // std::cout << "points : " << points_l_0.size() << " "<< points_r_0.size() << " "<< points_r_1.size() << " "<< points_l_1.size() << " "<<std::endl;
}

void bucketingFeatures(cv::Mat& image, FeatureSet& current_features, int bucket_size, int features_per_bucket)
{
// This function buckets features
// image: only use for getting dimension of the image
// bucket_size: bucket size in pixel is bucket_size*bucket_size
// features_per_bucket: number of selected features per bucket
    int image_height = image.rows;
    int image_width = image.cols;
    int buckets_nums_height = image_height/bucket_size;
    int buckets_nums_width = image_width/bucket_size;
    int buckets_number = buckets_nums_height * buckets_nums_width;

    std::vector<Bucket> Buckets;

    // initialize all the buckets
    for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height; buckets_idx_height++)
    {
      for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width; buckets_idx_width++)
      {
        Buckets.push_back(Bucket(features_per_bucket));
      }
    }

    // bucket all current features into buckets by their location
    int buckets_nums_height_idx, buckets_nums_width_idx, buckets_idx;
    for (int i = 0; i < current_features.points.size(); ++i)
    {
      buckets_nums_height_idx = current_features.points[i].y/bucket_size;
      buckets_nums_width_idx = current_features.points[i].x/bucket_size;
      buckets_idx = buckets_nums_height_idx*buckets_nums_width + buckets_nums_width_idx;
      Buckets[buckets_idx].add_feature(current_features.points[i], current_features.ages[i]);

    }

    // get features back from buckets
    current_features.clear();
    for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height; buckets_idx_height++)
    {
      for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width; buckets_idx_width++)
      {
         buckets_idx = buckets_idx_height*buckets_nums_width + buckets_idx_width;
         Buckets[buckets_idx].get_features(current_features);
      }
    }

    // std::cout << "current features number after bucketing: " << current_features.size() << std::endl;

}

void appendNewFeatures(cv::Mat& image, FeatureSet& current_features)
{
    std::vector<cv::Point2f>  points_new;
    featureDetectionFast(image, points_new);
    current_features.points.insert(current_features.points.end(), points_new.begin(), points_new.end());
    std::vector<int>  ages_new(points_new.size(), 0);
    current_features.ages.insert(current_features.ages.end(), ages_new.begin(), ages_new.end());
}