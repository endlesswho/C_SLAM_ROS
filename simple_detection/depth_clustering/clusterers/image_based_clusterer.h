// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
#define SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_

#include <chrono>
#include <ctime>
#include <map>
#include <opencv/cv.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/useful_typedefs.h"

#include "clusterers/abstract_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/cloud_projection.h"

namespace depth_clustering {

/**
 * @brief      Class for image based clusterer.
 *
 * @tparam     LabelerT  A Labeler class to be used for labeling.
 */
    template <typename LabelerT>
    class ImageBasedClusterer : public AbstractClusterer {
    public:
        using Receiver = AbstractClient<Cloud>;
        using Sender = AbstractSender<std::vector<Cloud>>;

        /**
         * @brief      Construct an image-based clusterer.
         *
         * @param[in]  angle_tollerance  The angle tollerance to separate objects
         * @param[in]  min_cluster_size  The minimum cluster size to send
         * @param[in]  max_cluster_size  The maximum cluster size to send
         */
        explicit ImageBasedClusterer(Radians angle_tollerance = 8_deg,
                                     uint16_t min_cluster_size = 100,
                                     uint16_t max_cluster_size = 125000)
                : AbstractClusterer(0.0, min_cluster_size, max_cluster_size),
                  _counter(0),
                  _angle_tollerance(angle_tollerance),
                  _label_client{nullptr} {}

        virtual ~ImageBasedClusterer() {}

        /**
         * @brief      Sets the difference type.
         *
         * @param[in]  diff_type  The difference type
         */
        void SetDiffType(DiffFactory::DiffType diff_type) { _diff_type = diff_type; }

        /**
         * @brief      Sets the label image client.
         *
         * @param      client  The client to receive color images with labels
         */
        void SetLabelImageClient(AbstractClient<cv::Mat>* client) {
          this->_label_client = client;
        }

        /**
         * @brief      Gets called when clusterer receives a cloud to cluster
         *
         * @param[in]  cloud      The cloud to cluster
         * @param[in]  sender_id  The sender identifier
         */
        void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override {
          // generate a projection from a point cloud
          if (!cloud.projection_ptr()) {
            fprintf(stderr, "ERROR: projection not initialized in cloud.\n");
            fprintf(stderr, "INFO: cannot label this cloud.\n");
            return;
          }
          time_utils::Timer timer;
          LabelerT image_labeler(cloud.projection_ptr()->depth_image(),
                                 cloud.projection_ptr()->params(), _angle_tollerance);
          image_labeler.ComputeLabels(_diff_type);

//    cv::Mat edge_result;
//    cv::Mat src;
//    src = cloud.projection_ptr()->depth_image();
//    std::cout<<src.channels()<<std::endl;
//    cv::Mat canny_mat(src.size(),CV_8U);
//    cv::Laplacian(src,edge_result,src.depth());

//   cv::Canny(canny_mat,edge_result,50,150,3);
//   cv::imwrite( "/home/huyh/depth_clustering/Canny_Image.jpg", canny_mat );
//   cv::imwrite( "/home/huyh/depth_clustering/Edge_Image.jpg", edge_result );
//   cv::imwrite( "/home/huyh/depth_clustering/Gray_Image.jpg", cloud.projection_ptr()->depth_image() );//cloud.projection_ptr->depth_image() is a cv::Mat
//    std::cout<<"!!!!!!!!!!!!!images have been written!!!!!!!!!!!!!!!!!!!!"<<std::endl;

          const cv::Mat* labels_ptr = image_labeler.GetLabelImage();
          fprintf(stderr, "INFO: image based labeling took: %lu us\n",
                  timer.measure());

          // send image to whoever wants to get it
          if (_label_client) {
            _label_client->OnNewObjectReceived(*labels_ptr, this->id());
          }

          // create 3d clusters from image labels
          std::unordered_map<uint16_t, Cloud> clusters;
          for (int row = 0; row < labels_ptr->rows; ++row) {
            for (int col = 0; col < labels_ptr->cols; ++col) {
              const auto& point_container = cloud.projection_ptr()->at(row, col);
              if (point_container.IsEmpty()) {
                // this is ok, just continue, nothing interesting here, no points.
                continue;
              }
              uint16_t label = labels_ptr->at<uint16_t>(row, col);
              if (label < 1) {
                // this is a default label, skip
                continue;
              }
              for (const auto& point_idx : point_container.points()) {
                const auto& point = cloud.points()[point_idx];
                clusters[label].push_back(point);
              }
            }
          }

          std::cout<<"the size of clusters"<<clusters.size()<<std::endl;

          // now send clusters to clients. They can't wait to get a new cloud.
          auto labels_color = AbstractImageLabeler::LabelsToColor(*labels_ptr);
          _counter++;
//    std::vector<Cloud> clusters_to_send;
          clusters_to_send.clear();
          for (auto& kv : clusters) {
            // if (counter > num_clusters_max) { break; }
            auto& cluster = kv.second;
            // add an rule to save the proper clusters
            Eigen::Vector3f center = Eigen::Vector3f::Zero();
            Eigen::Vector3f extent = Eigen::Vector3f::Zero();
            Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest());
            Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max());
            for (const auto& point : cluster.points()) {
              center = center + point.AsEigenVector();
              min_point << std::min(min_point.x(), point.x()),
                      std::min(min_point.y(), point.y()),
                      std::min(min_point.z(), point.z());
              max_point << std::max(max_point.x(), point.x()),
                      std::max(max_point.y(), point.y()),
                      std::max(max_point.z(), point.z());
            }
            if (min_point.x() < max_point.x()) {
              extent = max_point - min_point;
            }
//      std::cout<<extent<<std::endl;
//      std::cout<<std::endl;

//      if (extent.x()<0.5 ||
//              extent.y()<0.5 ||
//              extent.z()<0.5){
//          continue;
//        // cluster size is too small or too big, forget it
//      }

//        if (extent.x()<0.5 ||
//            extent.y()<0.5 ||
//            extent.z()<1){
//            continue;
//            // cluster size is too small or too big, forget it
//        }

            if (cluster.size() < this->_min_cluster_size ||
                cluster.size() > this->_max_cluster_size) {
              // cluster is too small or too big, forget it.
              continue;
            }
            cluster.InitProjection(cloud.projection_ptr()->params());
            clusters_to_send.push_back(cluster);
          }

          this->ShareDataWithAllClients(clusters);
        }

        std::vector<Cloud> getClusters(){
          std::cout<<"the size of clusters_to_send"<<clusters_to_send.size()<<std::endl;
          return clusters_to_send;
        }

    private:
        int _counter;
        std::vector<Cloud> clusters_to_send;
        Radians _angle_tollerance;

        AbstractClient<cv::Mat>* _label_client;

        DiffFactory::DiffType _diff_type = DiffFactory::DiffType::NONE;
    };

}  // namespace depth_clustering

#endif  // SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_