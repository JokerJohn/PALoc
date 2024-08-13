//
// Created by xchu on 28/3/2023.
//

#ifndef SRC_GRAVITY_FATOR2_H
#define SRC_GRAVITY_FATOR2_H

#endif //SRC_GRAVITY_FATOR2_H
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/base/Matrix.h>

using namespace gtsam;


 namespace gtsam{
//     class GravityFactor2 : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
//     public:
//         GravityFactor2(const gtsam::Key &pose_key, const gtsam::Key &gravity_key,
//                        const Eigen::Vector3d &acc_measured_gravity,
//                        const gtsam::SharedNoiseModel &model)
//                 : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(model, pose_key, gravity_key),
//                   acc_measured_gravity_(acc_measured_gravity) {}
//
//         gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector3 &gravity,
////                                     boost::optional<gtsam::Matrix &> H1 = boost::none,
//                                     boost::optional<gtsam::Matrix &> H = boost::none) const override {
//             Eigen::Vector3d a_measured_world = pose.rotation().matrix() * acc_measured_gravity_;
//             Eigen::Vector3d a_measured_direction = a_measured_world.normalized();
//
//             // Direction error
//             gtsam::Vector3 expected_accel = -gravity.normalized();
//             gtsam::Vector3 e_dir = a_measured_direction - expected_accel;
//
//             // Magnitude error
//             double e_norm = a_measured_world.norm() / gravity.norm() - 1.0;
//
//             // Error vector
//             gtsam::Vector4 error;
//             error.head<3>() = e_dir;
//             error(3) = e_norm;
//
////             if (H1 || H2) {
////                 // Calculate the Jacobian matrix with respect to the pose
////                 gtsam::Matrix43 J_pose;
////                 J_pose.block<3, 3>(0, 0) = gtsam::Matrix3::Zero();  // No dependency on translation
////                 J_pose.block<3, 3>(0, 3) = pose.rotation().matrix() * gtsam::skewSymmetric(acc_measured_gravity_);
////                 J_pose.block<1, 3>(3, 0) = gtsam::Matrix13::Zero(); // No dependency on translation
////                 J_pose.block<1, 3>(3, 3) =
////                         (a_measured_world.transpose() / a_measured_world.norm()) * pose.rotation().matrix() *
////                         gtsam::skewSymmetric(acc_measured_gravity_);
////
////                 if (H1) *H1 = J_pose;
////
////                 // Calculate the Jacobian matrix with respect to the gravity
////                 gtsam::Matrix34 J_gravity;
////                 J_gravity.block<3, 3>(0, 0) = -gtsam::Matrix3::Identity();
////                 J_gravity.block<3, 1>(0, 3) = gtsam::Vector3::Zero();
////                 J_gravity.block<1, 3>(3, 0) = (a_measured_world.transpose() / (a_measured_world.norm() * gravity.norm()));
////                 J_gravity(3, 3) = -a_measured_world.norm() / (gravity.norm() * gravity.norm());
////
////                 if (H2) *H2 = J_gravity;
//
//
//             if (H){
//                 gtsam::Matrix49 J;
//                 J.block<3, 3>(0, 0) = gtsam::Matrix3::Zero();  // No dependency on translation
//                 J.block<3, 3>(0, 3) = pose.rotation().matrix() * gtsam::skewSymmetric(acc_measured_gravity_);
//                 J.block<1, 3>(3, 0) = gtsam::Matrix13::Zero(); // No dependency on translation
//                 J.block<1, 3>(3, 3) =
//                         (a_measured_world.transpose() / a_measured_world.norm()) * pose.rotation().matrix() *
//                         gtsam::skewSymmetric(acc_measured_gravity_);
//                 J.block<3, 3>(0, 6) = gtsam::Matrix3::Identity();
//                 J.block<1, 3>(3, 6) = gtsam::Matrix13::Zero();
//                 *H = J;
//             }
//
//
//
//             return error;
//         }
//
//         virtual gtsam::NonlinearFactor::shared_ptr clone() const {
//             return boost::make_shared<GravityFactor2>(*this);
//         }
//
//     private:
//         gtsam::Vector3 acc_measured_gravity_;
//     };




    class GravityFactor2 : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
    public:
        GravityFactor2(const gtsam::Key &pose_key, const gtsam::Key &gravity_key,
                      const Eigen::Vector3d &acc_measured_gravity,
                      const gtsam::SharedNoiseModel &model)
                : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(model, pose_key, gravity_key),
                  acc_measured_gravity_(acc_measured_gravity) {}

        gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector3 &gravity,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none,
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const override {

            Eigen::Vector3d a_measured_world = pose.rotation().matrix() * acc_measured_gravity_;
            Eigen::Vector3d a_measured_direction = a_measured_world.normalized();

            // Direction error
            gtsam::Vector3 expected_accel(0, 0, -1);
            gtsam::Vector3 e_dir = a_measured_direction - expected_accel;

            // Magnitude error
            double e_norm = a_measured_world.norm() / 9.81 - 1.0;

            // Error vector
            gtsam::Vector4 error;
            error.head<3>() = e_dir;
            error(3) = e_norm;

            if (H1) {
//                gtsam::Matrix44 J1;
                gtsam::Matrix44 J1 = gtsam::Matrix44::Zero();  // Initialize with zeros

//                gtsam::Matrix43 J1;
                J1.block<3, 3>(0, 0) = gtsam::Matrix3::Zero();  // No dependency on translation
                J1.block<3, 3>(0, 3) = pose.rotation().matrix() * gtsam::skewSymmetric(acc_measured_gravity_);
                J1.block<1, 3>(3, 0) = gtsam::Matrix13::Zero(); // No dependency on translation
                J1.block<1, 3>(3, 3) =
                        (a_measured_world.transpose() / a_measured_world.norm()) * pose.rotation().matrix() *
                        gtsam::skewSymmetric(acc_measured_gravity_);
                *H1 = J1;
            }

            if (H2) {
//                gtsam::Matrix44 J2;
//                gtsam::Matrix43 J2;
                gtsam::Matrix44 J2 = gtsam::Matrix44::Zero();  // Initialize with zeros

                J2.block<3, 3>(0, 0) = gtsam::Matrix3::Identity();
                J2.block<1, 3>(3, 0) = gtsam::Matrix13::Zero();
                *H2 = J2;
            }

            return error;
        }

        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::make_shared<GravityFactor2>(*this);
        }

    private:
        gtsam::Vector3 acc_measured_gravity_;
    };

}
