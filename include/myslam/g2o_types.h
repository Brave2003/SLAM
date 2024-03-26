#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "myslam/common_include.h"

/**
 * @brief 
 * VertexPose 类：这是一个6自由度的位姿顶点，表示一个刚体在三维空间中的位置和朝向。它继承自g2o::BaseVertex<6, SE3>，其中SE3是一个表示三维刚体变换的李群。VertexPose重写了setToOriginImpl()、oplusImpl()等方法，分别用于设置顶点的初始估计值和更新顶点的估计值。
 * VertexXYZ 类：这是一个三维空间点的顶点，表示地图中的一个路标。它继承自g2o::BaseVertex<3, Vec3>，其中Vec3是一个三维向量。VertexXYZ同样重写了setToOriginImpl()和oplusImpl()等方法。
 * EdgeProjectionPoseOnly 类：这是一个只估计位姿的一元边，它连接一个VertexPose顶点。在SLAM中，这种边可能用于表示只有位姿信息而没有地图点的观测。它继承自g2o::BaseUnaryEdge<2, Vec2, VertexPose>，表示该边测量的是一个二维向量（如图像上的像素坐标），连接的是一个位姿顶点。EdgeProjectionPoseOnly重写了computeError()和linearizeOplus()等方法，分别用于计算误差和线性化误差函数。
 * EdgeProjection 类：这是一个带有地图点和位姿的二元边，它连接一个VertexPose顶点和一个VertexXYZ顶点。在SLAM中，这种边通常用于表示一个地图点在相机图像上的观测。它继承自g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>，同样重写了computeError()和linearizeOplus()等方法。
 * 
 */
namespace myslam {
    class VertexPose : public g2o::BaseVertex<6, SE3>{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            virtual void setToOriginImpl() override{
                _estimate = SE3();
            };

            /**
             * @brief 输入六向量数组，通过指数映射，更新位姿
             * 
             * @param update 
             */
            virtual void oplusImpl(const double *update) override{
                Vec6 update_eigen;
                update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
                _estimate = SE3::exp(update_eigen)* _estimate;
            }

            virtual bool read(std::istream &in) override {
                return true;
            }

            virtual bool write(std::ostream &out) const override {
                return true;
            }

    };

    class VertexXYZ : public g2o::BaseVertex<3, Vec3> {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            virtual void setToOriginImpl() override{
                _estimate = Vec3::Zero();
            }

            virtual void oplusImpl(const double *update) override{
                _estimate[0] += update[0];
                _estimate[1] += update[1];
                _estimate[2] += update[2];
            }

            virtual bool read(std::istream &in) override {
                return true;
            }

            virtual bool write(std::ostream &out) const override {
                return true;
            }
    };
    /**
     * @brief BaseUnaryEdge<2, Vec2, VertexPose>,2表示边的维度数，Vec2表示测量值， Pose表示优化点
     *
     */
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose>{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K): _pos3d(pos), _K(K){}

            // _vertices 存储与当前边相关联的顶点的指针
            virtual void computeError() override{
                // 使用第一个顶点，因为边为一元边
                const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
                SE3 T = v->estimate();
                // 因为该类只考虑左图来计算位姿 所以不需要进行外参的变化,左图相机是基准相机
                Vec3 pos_pixel = _K * ( T * _pos3d );
                pos_pixel /= pos_pixel[2]; // (x/z, y/z, 1)
                _error = _measurement - pos_pixel.head<2>();
            }

            virtual void linearizeOplus() override{
                const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
                SE3 T = v->estimate();
                Vec3 pos_cam = T * _pos3d;
                double fx = _K(0, 0);
                double fy = _K(1, 1);
                double X = pos_cam[0];
                double Y = pos_cam[1];
                double Z = pos_cam[2];
                // 防止Z==0
                double Zinv = 1.0 / (Z + 1e-18);
                double Zinv2 = Zinv * Zinv;
                // don't use _jacobianOplus 因为需要计算一个定点优化的雅克比矩阵
                _jacobianOplusXi << -fx*Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;

            }

            virtual bool read(std::istream &in) override {
                return true;
            }

            virtual bool write(std::ostream &out) const override {
                return true;
            }


        private:
            // 相机坐标系下位置
            Vec3 _pos3d;
            Mat33 _K;
    };

    class EdgeProjection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeProjection(const Mat33 &K, const SE3 &cam_ext): _K(K), _cam_ext(cam_ext){};

            virtual void computeError() override{
                const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
                const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
                SE3 T = v0 -> estimate();
                // 因为该类需要考虑左右图
                Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
                pos_pixel /= pos_pixel[2]; // (x/z, y/z, 1)
                _error = _measurement - pos_pixel.head<2>();
            }

            virtual void linearizeOplus() override{
                const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
                const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
                SE3 T = v0 -> estimate();
                Vec3 pw = v1 -> estimate();
                Vec3 pos_cam = _cam_ext * T * pw;
                double fx = _K(0, 0);
                double fy = _K(1, 1);
                double X = pos_cam[0];
                double Y = pos_cam[1];
                double Z = pos_cam[2];
                double Zinv = 1.0 / (Z + 1e-18);
                double Zinv2 = Zinv * Zinv;
                _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;
                // Xi 是因为有有个自由度可以优化， Yi 只有三个 所以是 2×6 和 2×3
                // 因为要考虑左右图外参
                _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                                _cam_ext.rotationMatrix() * T.rotationMatrix();

            }

            virtual bool read(std::istream &in) override { return true; }

            virtual bool write(std::ostream &out) const override { return true; }
        private:
            Mat33 _K;
            SE3 _cam_ext;
    };
}
#endif
