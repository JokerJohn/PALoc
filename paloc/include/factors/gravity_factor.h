//
// Created by xchu on 21/3/2023.
//

#ifndef SRC_GRAVITYFACTOR_H
#define SRC_GRAVITYFACTOR_H


#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;

namespace gtsam {


    // 重力因子
    class GravityFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    public:
        GravityFactor(const gtsam::Key &pose_key,
                      const Eigen::Vector3d &acc_measured_gravity,
                      const gtsam::SharedNoiseModel &model)
                : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
                  acc_measured_gravity_(acc_measured_gravity) {}

        // 计算重力约束的误差
        gtsam::Vector
        evaluateError(const gtsam::Pose3 &pose, boost::optional<gtsam::Matrix &> H = boost::none) const override {
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

            if (H) {
                // Calculate the Jacobian matrix with respect to the pose
                gtsam::Matrix46 J;
//                J.block<3, 3>(0, 0) = gtsam::Matrix3::Zero();  // No dependency on translation
//                J.block<3, 3>(0, 3) = pose.rotation().matrix() * gtsam::skewSymmetric(acc_measured_gravity_);
//                J.block<1, 3>(3, 0) = gtsam::Matrix13::Zero(); // No dependency on translation
//                J.block<1, 3>(3, 3) =
//                        (a_measured_world.transpose() / a_measured_world.norm()) * pose.rotation().matrix() *
//                        gtsam::skewSymmetric(acc_measured_gravity_);
                J.block<3, 3>(0, 0) = gtsam::Matrix3::Zero();  // No dependency on translation
                J.block<3, 3>(0, 3) = -pose.rotation().matrix() * gtsam::skewSymmetric(acc_measured_gravity_) /
                                      a_measured_world.norm();
                J.block<1, 3>(3, 0) = gtsam::Matrix13::Zero(); // No dependency on translation
                J.block<1, 3>(3, 3) = gtsam::Matrix13::Zero();
                *H = J;
            }

//            std::cout << "a_measured_world: " << a_measured_world.transpose() << std::endl;
//            std::cout << "a_measured_direction: " << a_measured_direction.transpose() << std::endl;
//            std::cout << "error dir: " << error.transpose() << std::endl;
            // std::cout << "error e_norm: " << e_norm << std::endl;
            return error;
        }


        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::make_shared<GravityFactor>(*this);
        }


    private:
        gtsam::Vector3 acc_measured_gravity_;
    };


    class GravityFactor2 : public NoiseModelFactor1<Pose3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector3 measured_acc_gravity_;

        GravityFactor2(Key poseKey, const Vector3 &measured_acc_gravity, const SharedNoiseModel &model) :
                NoiseModelFactor1<Pose3>(model, poseKey), measured_acc_gravity_(measured_acc_gravity) {}

        Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H = boost::none) const override {
            // World frame acceleration vector a_w
            Vector3 a_w = pose.rotation().matrix() * measured_acc_gravity_;
            Vector3 a_w_normalized = a_w.normalized();
            // Adjust the expected gravity direction based on the IMU mounting direction
            Vector3 expected_accel(0, 0, measured_acc_gravity_.z() > 0 ? -1.0 : 1.0);
            // Direction error
            Vector3 e_dir = a_w_normalized - expected_accel;
            // Magnitude error
            double e_mag = a_w.norm() - 9.81;
            // Combine errors
            Vector4 error;
            error << e_dir, e_mag;

            if (H) {
                // The Jacobian has 4 rows to match the error vector
                Matrix J(4, 6);
                J.setZero();
                // Jacobian block for direction error
                Matrix36 J_dir;
                J_dir.setZero();
                J_dir.block<3, 3>(0, 3) = -pose.rotation().matrix() * skewSymmetric(measured_acc_gravity_) / a_w.norm();
                // Jacobian block for magnitude error, only rotation part affects the magnitude
                Eigen::RowVector3d J_mag = a_w.transpose() / a_w.norm();
                // Fill in the Jacobian matrix
                J.block<3, 6>(0, 0) = J_dir;
                J.block<1, 3>(3, 0).setZero(); // translation part is zero
                J.block<1, 3>(3, 3) = J_mag; // rotation part
                *H = J;
            }
            return error;
        }

        static Matrix3 skewSymmetric(const Vector3 &v) {
            Matrix3 m;
            m << 0, -v.z(), v.y(),
                    v.z(), 0, -v.x(),
                    -v.y(), v.x(), 0;
            return m;
        }

    };


    class GravityFactorAuto : public NoiseModelFactor1<Pose3> {
    private:
        Vector3 measured_acc_gravity_;
    public:
        GravityFactorAuto(Key key, const Vector3 &gravity, const SharedNoiseModel &model)
                : NoiseModelFactor1<Pose3>(model, key), measured_acc_gravity_(gravity) {}

        Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H = boost::none) const override {
            if (H) {
                *H = numericalDerivative11<Vector, Pose3>(
                        boost::bind(&GravityFactorAuto::evaluateError, this, _1, boost::none), pose, 1e-5);
            }
            Vector3 a_w = pose.rotation().matrix() * measured_acc_gravity_;
            Vector3 a_w_normalized = a_w.normalized();
            Vector3 expected_accel(0, 0, -1.0);
            Vector3 e_dir = a_w_normalized - expected_accel;
            double e_mag = a_w.norm() - 9.81;
            Vector4 error;
            error << e_dir, e_mag;
            return error;
        }
    };

    // 自定义重力因子
    class StaticGravityFactor : public NoiseModelFactor1<Pose3> {
    private:
        Vector3 measured_acc_gravity_; // 测量到的加速度，即重力
    public:
        // 构造函数
        StaticGravityFactor(Key key, const Vector3 &measured_acc_gravity, const SharedNoiseModel &model)
                : NoiseModelFactor1<Pose3>(model, key), measured_acc_gravity_(measured_acc_gravity) {}

        // 计算误差
        Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H = boost::none) const override {
            if (H) {
                *H = numericalDerivative11<Vector, Pose3>(
                        boost::bind(&StaticGravityFactor::evaluateError, this, _1, boost::none), pose, 1e-5);
            }
            Vector3 a_w = pose.rotation().matrix() * measured_acc_gravity_;
            Vector3 a_w_normalized = a_w.normalized();
            Vector3 expected_accel(0, 0, -1.0); // 假设重力向量是单位向下的
            Vector3 e_dir = a_w_normalized - expected_accel;
            return e_dir; // 仅返回方向误差
        }
    };
}


#endif //SRC_GRAVITYFACTOR_H
